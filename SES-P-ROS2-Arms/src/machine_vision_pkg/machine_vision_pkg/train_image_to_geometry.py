import argparse
import copy
import json
import os
import random
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from PIL import Image
from sklearn.metrics import classification_report, confusion_matrix
from torch.utils.data import DataLoader, Dataset
from torchvision import datasets, models, transforms


ASSET_EXTENSIONS = {".glb", ".gltf", ".ply", ".obj", ".stl", ".off"}


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False


def build_transforms(image_size: int, strong_aug: bool) -> Tuple[transforms.Compose, transforms.Compose]:
    if strong_aug:
        train_tf = transforms.Compose([
            transforms.Resize((image_size + 32, image_size + 32)),
            transforms.RandomResizedCrop(image_size, scale=(0.72, 1.0), ratio=(0.9, 1.1)),
            transforms.RandomHorizontalFlip(),
            transforms.RandomRotation(12),
            transforms.RandomApply([transforms.GaussianBlur(kernel_size=5, sigma=(0.1, 1.4))], p=0.2),
            transforms.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.22, hue=0.05),
            transforms.ToTensor(),
            transforms.RandomErasing(p=0.12, scale=(0.02, 0.08), ratio=(0.4, 2.5), value="random"),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
    else:
        train_tf = transforms.Compose([
            transforms.Resize((image_size, image_size)),
            transforms.RandomHorizontalFlip(),
            transforms.RandomRotation(8),
            transforms.ColorJitter(brightness=0.15, contrast=0.15, saturation=0.12, hue=0.03),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])

    eval_tf = transforms.Compose([
        transforms.Resize((image_size, image_size)),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
    ])
    return train_tf, eval_tf


class SubsetImageFolder(Dataset):
    def __init__(self, base_samples: List[Tuple[str, int]], transform, repeat_factor: int = 1):
        self.samples = base_samples
        self.transform = transform
        self.repeat_factor = max(1, repeat_factor)

    def __len__(self) -> int:
        return len(self.samples) * self.repeat_factor

    def __getitem__(self, index: int):
        index = index % len(self.samples)
        path, label = self.samples[index]
        image = Image.open(path).convert("RGB")
        if self.transform is not None:
            image = self.transform(image)
        return image, label


def group_samples_by_class(samples: Sequence[Tuple[str, int]]) -> Dict[int, List[str]]:
    grouped: Dict[int, List[str]] = defaultdict(list)
    for path, label in samples:
        grouped[label].append(path)
    return grouped


def split_samples_per_class(
    dataset: datasets.ImageFolder,
    train_ratio: float,
    seed: int,
) -> Tuple[List[Tuple[str, int]], List[Tuple[str, int]]]:
    rng = random.Random(seed)
    grouped = group_samples_by_class(dataset.samples)
    train_samples: List[Tuple[str, int]] = []
    val_samples: List[Tuple[str, int]] = []

    for label, paths in grouped.items():
        if len(paths) < 2:
            raise ValueError(f"Class index {label} needs at least 2 images for train/val split.")
        shuffled = paths[:]
        rng.shuffle(shuffled)
        split_idx = max(1, int(len(shuffled) * train_ratio))
        split_idx = min(split_idx, len(shuffled) - 1)
        train_samples.extend((path, label) for path in shuffled[:split_idx])
        val_samples.extend((path, label) for path in shuffled[split_idx:])

    return train_samples, val_samples


def split_real_support_query(
    real_dataset: datasets.ImageFolder,
    k_shot: int,
    val_per_class: int,
    test_per_class: int,
    seed: int,
) -> Tuple[List[Tuple[str, int]], List[Tuple[str, int]], List[Tuple[str, int]]]:
    rng = random.Random(seed)
    grouped = group_samples_by_class(real_dataset.samples)

    support_samples: List[Tuple[str, int]] = []
    val_samples: List[Tuple[str, int]] = []
    test_samples: List[Tuple[str, int]] = []

    for label, paths in grouped.items():
        minimum_needed = k_shot + 2
        if len(paths) < minimum_needed:
            raise ValueError(
                f"Class index {label} has only {len(paths)} images. "
                f"You need at least {minimum_needed} images per class to build support/val/test."
            )

        shuffled = paths[:]
        rng.shuffle(shuffled)

        support_paths = shuffled[:k_shot]
        remaining = shuffled[k_shot:]

        if val_per_class > 0:
            current_val_count = min(val_per_class, len(remaining) - 1)
        else:
            current_val_count = max(1, len(remaining) // 2)

        val_paths = remaining[:current_val_count]
        leftover = remaining[current_val_count:]
        if len(leftover) == 0:
            raise ValueError(f"Class index {label} does not have enough images left for test data.")

        if test_per_class > 0:
            test_paths = leftover[: min(test_per_class, len(leftover))]
        else:
            test_paths = leftover

        support_samples.extend((path, label) for path in support_paths)
        val_samples.extend((path, label) for path in val_paths)
        test_samples.extend((path, label) for path in test_paths)

    return support_samples, val_samples, test_samples


def remap_samples_to_canonical(
    dataset: datasets.ImageFolder,
    canonical_classes: Sequence[str],
    dataset_name: str,
) -> List[Tuple[str, int]]:
    if set(dataset.classes) != set(canonical_classes):
        raise ValueError(
            f"{dataset_name} classes do not match canonical classes.\n"
            f"{dataset_name} classes: {dataset.classes}\n"
            f"Canonical classes: {list(canonical_classes)}"
        )

    canonical_to_idx = {name: idx for idx, name in enumerate(canonical_classes)}
    idx_to_name = {idx: name for name, idx in dataset.class_to_idx.items()}

    remapped_samples: List[Tuple[str, int]] = []
    for path, label_idx in dataset.samples:
        cls_name = idx_to_name[label_idx]
        remapped_samples.append((path, canonical_to_idx[cls_name]))

    return remapped_samples


def split_sample_list_per_class(
    samples: Sequence[Tuple[str, int]],
    first_ratio: float,
    seed: int,
) -> Tuple[List[Tuple[str, int]], List[Tuple[str, int]]]:
    if not 0.0 < first_ratio < 1.0:
        raise ValueError(f"first_ratio must be in (0, 1), got {first_ratio}")

    rng = random.Random(seed)
    grouped = group_samples_by_class(samples)
    first_samples: List[Tuple[str, int]] = []
    second_samples: List[Tuple[str, int]] = []

    for label, paths in grouped.items():
        if len(paths) < 2:
            raise ValueError(
                f"Class index {label} has only {len(paths)} images; need at least 2 to split into val/test."
            )

        shuffled = paths[:]
        rng.shuffle(shuffled)
        split_idx = int(round(len(shuffled) * first_ratio))
        split_idx = max(1, split_idx)
        split_idx = min(split_idx, len(shuffled) - 1)

        first_samples.extend((path, label) for path in shuffled[:split_idx])
        second_samples.extend((path, label) for path in shuffled[split_idx:])

    return first_samples, second_samples


def make_loader(dataset: Dataset, batch_size: int, shuffle: bool, num_workers: int) -> DataLoader:
    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers,
        pin_memory=torch.cuda.is_available(),
    )


def is_asset_file(path: Path) -> bool:
    return path.is_file() and path.suffix.lower() in ASSET_EXTENSIONS


def discover_assets(asset_dir: str) -> Dict[str, List[str]]:
    root = Path(asset_dir)
    if not root.is_dir():
        raise FileNotFoundError(f"Asset directory not found: {asset_dir}")

    assets_by_class: Dict[str, List[str]] = defaultdict(list)
    top_level_files = [p for p in root.iterdir() if is_asset_file(p)]

    if top_level_files:
        for path in sorted(top_level_files):
            assets_by_class[path.stem].append(str(path))

    for child in sorted(root.iterdir()):
        if not child.is_dir():
            continue
        class_assets = [str(p) for p in sorted(child.rglob("*")) if is_asset_file(p)]
        if class_assets:
            assets_by_class[child.name].extend(class_assets)

    if not assets_by_class:
        raise ValueError(f"No supported 3D asset files found under {asset_dir}")

    return dict(assets_by_class)


def normalize_scene_geometry(trimesh_scene):
    import trimesh

    if isinstance(trimesh_scene, trimesh.Trimesh):
        mesh = trimesh_scene.copy()
    elif isinstance(trimesh_scene, trimesh.Scene):
        if hasattr(trimesh_scene, "to_geometry"):
            mesh = trimesh_scene.to_geometry()
        else:
            mesh = trimesh_scene.dump(concatenate=True)
    else:
        raise TypeError(f"Unsupported trimesh type: {type(trimesh_scene)}")

    if mesh.vertices.shape[0] == 0:
        raise ValueError("Loaded mesh has no vertices")

    bounds = mesh.bounds
    center = (bounds[0] + bounds[1]) / 2.0
    extents = bounds[1] - bounds[0]
    size = float(np.max(extents))
    if size <= 0:
        raise ValueError("Loaded mesh has invalid bounding box size")

    mesh.vertices = mesh.vertices - center
    mesh.vertices = mesh.vertices / size
    return mesh


def canonicalize_signs(points: np.ndarray, normals: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    points = points.copy()
    normals = normals.copy()
    for axis in range(3):
        skew = float(np.mean(points[:, axis] ** 3))
        if skew < 0:
            points[:, axis] *= -1.0
            normals[:, axis] *= -1.0
    return points, normals


def hist1d(values: np.ndarray, bins: int, value_range: Tuple[float, float]) -> np.ndarray:
    hist, _ = np.histogram(values, bins=bins, range=value_range)
    hist = hist.astype(np.float32)
    total = float(hist.sum())
    if total > 0:
        hist /= total
    return hist


def extract_mesh_descriptor(
    asset_path: str,
    num_surface_points: int,
    axis_bins: int,
    radial_bins: int,
    pairwise_bins: int,
) -> np.ndarray:
    import trimesh

    loaded = trimesh.load(asset_path, force="scene")
    mesh = normalize_scene_geometry(loaded)

    if len(mesh.faces) == 0:
        raise ValueError(f"Mesh has no faces: {asset_path}")

    points, face_indices = trimesh.sample.sample_surface(mesh, num_surface_points)
    points = points.astype(np.float32)
    normals = mesh.face_normals[face_indices].astype(np.float32)

    point_norms = np.linalg.norm(points, axis=1)
    scale = float(np.percentile(point_norms, 95))
    if scale <= 1e-8:
        raise ValueError(f"Invalid sampled point scale for asset: {asset_path}")

    points = points / scale

    covariance = np.cov(points.T)
    eigvals, eigvecs = np.linalg.eigh(covariance)
    order = np.argsort(eigvals)[::-1]
    eigvals = np.maximum(eigvals[order], 1e-12)
    eigvecs = eigvecs[:, order]

    aligned_points = points @ eigvecs
    aligned_normals = normals @ eigvecs
    aligned_points, aligned_normals = canonicalize_signs(aligned_points, aligned_normals)

    aligned_normals /= np.linalg.norm(aligned_normals, axis=1, keepdims=True).clip(min=1e-12)

    extents = aligned_points.max(axis=0) - aligned_points.min(axis=0)
    extents = extents / max(float(extents.max()), 1e-12)
    eig_ratios = eigvals / float(eigvals.sum())

    axis_hists = [
        hist1d(aligned_points[:, axis], bins=axis_bins, value_range=(-1.0, 1.0))
        for axis in range(3)
    ]
    radial_hist = hist1d(np.linalg.norm(aligned_points, axis=1), bins=radial_bins, value_range=(0.0, 1.6))

    subset_size = min(256, aligned_points.shape[0])
    subset = aligned_points[:subset_size]
    diffs = subset[:, None, :] - subset[None, :, :]
    dists = np.linalg.norm(diffs, axis=-1)
    pairwise = dists[np.triu_indices(subset_size, k=1)]
    pairwise_hist = hist1d(pairwise, bins=pairwise_bins, value_range=(0.0, 2.5))

    abs_normal_mean = np.mean(np.abs(aligned_normals), axis=0)
    abs_normal_std = np.std(np.abs(aligned_normals), axis=0)

    descriptor = np.concatenate(
        axis_hists
        + [radial_hist, pairwise_hist, eig_ratios.astype(np.float32), extents.astype(np.float32),
           abs_normal_mean.astype(np.float32), abs_normal_std.astype(np.float32)],
        axis=0,
    ).astype(np.float32)

    descriptor /= np.linalg.norm(descriptor).clip(min=1e-12)
    return descriptor


def build_geometry_prototypes(
    asset_dir: str,
    canonical_classes: Sequence[str],
    num_surface_points: int,
    axis_bins: int,
    radial_bins: int,
    pairwise_bins: int,
    device: torch.device,
) -> torch.Tensor:
    assets_by_class = discover_assets(asset_dir)

    missing = [name for name in canonical_classes if name not in assets_by_class]
    if missing:
        raise ValueError(f"Missing 3D assets for classes: {missing}")

    prototypes: List[torch.Tensor] = []
    for class_name in canonical_classes:
        asset_paths = assets_by_class[class_name]
        descriptors = [
            extract_mesh_descriptor(
                asset_path,
                num_surface_points=num_surface_points,
                axis_bins=axis_bins,
                radial_bins=radial_bins,
                pairwise_bins=pairwise_bins,
            )
            for asset_path in asset_paths
        ]
        descriptor = np.mean(np.stack(descriptors, axis=0), axis=0)
        descriptor /= np.linalg.norm(descriptor).clip(min=1e-12)
        prototypes.append(torch.tensor(descriptor, dtype=torch.float32, device=device))

    return torch.stack(prototypes, dim=0)


class ImageToGeometryModel(nn.Module):
    def __init__(self, backbone: str, geometry_dim: int, projector_hidden_dim: int, dropout: float):
        super().__init__()
        self.backbone_name = backbone

        if backbone == "resnet18":
            base = models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1)
            feature_dim = base.fc.in_features
            base.fc = nn.Identity()
        elif backbone == "convnext_tiny":
            base = models.convnext_tiny(weights=models.ConvNeXt_Tiny_Weights.IMAGENET1K_V1)
            feature_dim = base.classifier[2].in_features
            base.classifier[2] = nn.Identity()
        else:
            raise ValueError(f"Unsupported backbone: {backbone}")

        hidden_dim = projector_hidden_dim if projector_hidden_dim > 0 else max(256, feature_dim // 2)
        self.backbone = base
        self.projector = nn.Sequential(
            nn.Linear(feature_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, geometry_dim),
        )

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        features = self.backbone(images)
        projected = self.projector(features)
        return nn.functional.normalize(projected, dim=-1)


def configure_trainable_params(model: ImageToGeometryModel, mode: str) -> None:
    for param in model.parameters():
        param.requires_grad = False

    for param in model.projector.parameters():
        param.requires_grad = True

    if mode == "projector_only":
        return

    if mode == "last_block":
        if model.backbone_name == "resnet18":
            for param in model.backbone.layer4.parameters():
                param.requires_grad = True
        elif model.backbone_name == "convnext_tiny":
            for param in model.backbone.features[-1].parameters():
                param.requires_grad = True
        return

    if mode == "full":
        for param in model.parameters():
            param.requires_grad = True
        return

    raise ValueError(f"Unsupported train mode: {mode}")


def get_optimizer(model: nn.Module, lr: float, weight_decay: float) -> optim.Optimizer:
    params = [p for p in model.parameters() if p.requires_grad]
    return optim.AdamW(params, lr=lr, weight_decay=weight_decay)


def compute_losses(
    embeddings: torch.Tensor,
    labels: torch.Tensor,
    geometry_prototypes: torch.Tensor,
    logit_scale: float,
    align_weight: float,
) -> Tuple[torch.Tensor, torch.Tensor]:
    logits = logit_scale * (embeddings @ geometry_prototypes.t())
    ce_loss = nn.functional.cross_entropy(logits, labels)
    aligned = torch.sum(embeddings * geometry_prototypes[labels], dim=1)
    align_loss = (1.0 - aligned).mean()
    loss = ce_loss + align_weight * align_loss
    return loss, logits


def run_epoch(
    model: ImageToGeometryModel,
    loader: DataLoader,
    geometry_prototypes: torch.Tensor,
    optimizer: Optional[optim.Optimizer],
    device: torch.device,
    logit_scale: float,
    align_weight: float,
) -> Tuple[float, float]:
    train = optimizer is not None
    model.train(train)
    total_loss = 0.0
    total_correct = 0
    total_count = 0

    for images, labels in loader:
        images = images.to(device, non_blocking=True)
        labels = labels.to(device, non_blocking=True)

        with torch.set_grad_enabled(train):
            embeddings = model(images)
            loss, logits = compute_losses(embeddings, labels, geometry_prototypes, logit_scale, align_weight)
            if train:
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

        preds = logits.argmax(dim=1)
        total_loss += loss.item() * labels.size(0)
        total_correct += (preds == labels).sum().item()
        total_count += labels.size(0)

    avg_loss = total_loss / max(total_count, 1)
    avg_acc = total_correct / max(total_count, 1)
    return avg_loss, avg_acc


@torch.no_grad()
def evaluate_full(
    model: ImageToGeometryModel,
    loader: DataLoader,
    geometry_prototypes: torch.Tensor,
    device: torch.device,
    logit_scale: float,
    align_weight: float,
) -> Tuple[float, float, List[int], List[int], Dict[str, Dict[str, float]], List[List[int]]]:
    model.eval()
    total_loss = 0.0
    total_count = 0
    preds_all: List[int] = []
    labels_all: List[int] = []

    for images, labels in loader:
        images = images.to(device, non_blocking=True)
        labels = labels.to(device, non_blocking=True)

        embeddings = model(images)
        loss, logits = compute_losses(embeddings, labels, geometry_prototypes, logit_scale, align_weight)
        preds = logits.argmax(dim=1)

        total_loss += loss.item() * labels.size(0)
        total_count += labels.size(0)
        preds_all.extend(preds.cpu().tolist())
        labels_all.extend(labels.cpu().tolist())

    avg_loss = total_loss / max(total_count, 1)
    avg_acc = sum(int(p == y) for p, y in zip(preds_all, labels_all)) / max(total_count, 1)
    labels_range = list(range(geometry_prototypes.shape[0]))
    report = classification_report(labels_all, preds_all, labels=labels_range, output_dict=True, zero_division=0)
    cm = confusion_matrix(labels_all, preds_all, labels=labels_range).tolist()
    return avg_loss, avg_acc, labels_all, preds_all, report, cm


def fit_stage(
    stage_name: str,
    model: ImageToGeometryModel,
    train_loader: DataLoader,
    val_loader: DataLoader,
    geometry_prototypes: torch.Tensor,
    epochs: int,
    lr: float,
    weight_decay: float,
    device: torch.device,
    logit_scale: float,
    align_weight: float,
) -> Tuple[ImageToGeometryModel, Dict[str, float]]:
    optimizer = get_optimizer(model, lr=lr, weight_decay=weight_decay)

    best_state = copy.deepcopy(model.state_dict())
    best_val_acc = -1.0
    last_metrics: Dict[str, float] = {}

    for epoch in range(1, epochs + 1):
        train_loss, train_acc = run_epoch(
            model, train_loader, geometry_prototypes, optimizer, device, logit_scale, align_weight
        )
        val_loss, val_acc = run_epoch(
            model, val_loader, geometry_prototypes, optimizer=None, device=device, logit_scale=logit_scale,
            align_weight=align_weight
        )

        last_metrics = {
            "train_loss": train_loss,
            "train_acc": train_acc,
            "val_loss": val_loss,
            "val_acc": val_acc,
        }

        print(
            f"[{stage_name}] epoch {epoch:03d}/{epochs:03d} "
            f"train_loss={train_loss:.4f} train_acc={train_acc:.4f} "
            f"val_loss={val_loss:.4f} val_acc={val_acc:.4f}"
        )

        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_state = copy.deepcopy(model.state_dict())

    model.load_state_dict(best_state)
    return model, last_metrics


@dataclass
class RunSummary:
    classes: List[str]
    geometry_dim: int
    backbone: str
    train_mode: str
    synthetic_stage_enabled: bool
    synthetic_train_count: int
    synthetic_val_count: int
    real_support_count: int
    real_val_count: int
    real_test_count: int
    val_accuracy: float
    test_accuracy: float
    val_loss: float
    test_loss: float
    confusion_matrix: List[List[int]]
    classification_report: Dict[str, Dict[str, float]]
    model_path: str


def load_real_splits(
    args,
    canonical_classes: Sequence[str],
) -> Tuple[List[Tuple[str, int]], List[Tuple[str, int]], List[Tuple[str, int]], str]:
    use_explicit_real_dirs = bool(args.real_support_dir or args.real_test_dir or args.real_val_dir)

    if use_explicit_real_dirs:
        if not args.real_support_dir or not args.real_test_dir:
            raise ValueError(
                "When using explicit real-data mode, please provide both --real-support-dir and --real-test-dir."
            )
        if not os.path.isdir(args.real_support_dir):
            raise FileNotFoundError(f"Real support directory not found: {args.real_support_dir}")
        if not os.path.isdir(args.real_test_dir):
            raise FileNotFoundError(f"Real test directory not found: {args.real_test_dir}")
        if args.real_val_dir and not os.path.isdir(args.real_val_dir):
            raise FileNotFoundError(f"Real val directory not found: {args.real_val_dir}")

        real_support_base = datasets.ImageFolder(args.real_support_dir)
        support_samples = remap_samples_to_canonical(real_support_base, canonical_classes, "Real support")

        if args.real_val_dir:
            real_val_base = datasets.ImageFolder(args.real_val_dir)
            real_val_samples = remap_samples_to_canonical(real_val_base, canonical_classes, "Real validation")
            real_val_source = "separate real validation directory"
        else:
            if args.explicit_test_val_ratio > 0.0:
                real_test_base = datasets.ImageFolder(args.real_test_dir)
                full_test_samples = remap_samples_to_canonical(real_test_base, canonical_classes, "Real test")
                real_val_samples, real_test_samples = split_sample_list_per_class(
                    full_test_samples,
                    first_ratio=args.explicit_test_val_ratio,
                    seed=args.seed,
                )
                ratio_text = f"{args.explicit_test_val_ratio:.2f}".rstrip("0").rstrip(".")
                real_val_source = f"split from --real-test-dir with per-class val ratio={ratio_text}"
                return support_samples, real_val_samples, real_test_samples, real_val_source

            real_val_samples = support_samples[:]
            real_val_source = "support set reused for monitoring"

        real_test_base = datasets.ImageFolder(args.real_test_dir)
        real_test_samples = remap_samples_to_canonical(real_test_base, canonical_classes, "Real test")
        return support_samples, real_val_samples, real_test_samples, real_val_source

    if not os.path.isdir(args.real_dir):
        raise FileNotFoundError(
            f"Real data directory not found: {args.real_dir}\n"
            "Please put your real-life images into a matching ImageFolder directory."
        )

    real_base = datasets.ImageFolder(args.real_dir)
    if list(real_base.classes) != list(canonical_classes):
        raise ValueError(
            "Real dataset classes do not match canonical classes.\n"
            f"Real classes: {real_base.classes}\n"
            f"Canonical classes: {list(canonical_classes)}"
        )

    support_samples, real_val_samples, real_test_samples = split_real_support_query(
        real_base,
        k_shot=args.k_shot,
        val_per_class=args.real_val_per_class,
        test_per_class=args.real_test_per_class,
        seed=args.seed,
    )
    return support_samples, real_val_samples, real_test_samples, "automatic split from --real-dir"


def main() -> None:
    parser = argparse.ArgumentParser(description="Train an image encoder to align real images with 3D geometry prototypes")
    parser.add_argument("--asset-dir", type=str, required=True, help="Directory containing GLB/mesh assets")
    parser.add_argument("--synthetic-dir", type=str, default="", help="Optional ImageFolder root for synthetic images")
    parser.add_argument("--real-dir", type=str, default="./real_images_screws")
    parser.add_argument("--real-support-dir", type=str, default="")
    parser.add_argument("--real-val-dir", type=str, default="")
    parser.add_argument("--real-test-dir", type=str, default="")
    parser.add_argument(
        "--explicit-test-val-ratio",
        type=float,
        default=0.0,
        help="When using --real-support-dir + --real-test-dir without --real-val-dir, split the test dir per class into val/test using this ratio for val. Example: 0.2 means val:test = 2:8.",
    )
    parser.add_argument("--output-dir", type=str, default="./outputs_image_to_geometry")
    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--backbone",
        type=str,
        default="convnext_tiny",
        choices=["resnet18", "convnext_tiny"],
    )
    parser.add_argument(
        "--train-mode",
        type=str,
        default="projector_only",
        choices=["projector_only", "last_block", "full"],
    )
    parser.add_argument("--projector-hidden-dim", type=int, default=512)
    parser.add_argument("--projector-dropout", type=float, default=0.1)
    parser.add_argument("--k-shot", type=int, default=5)
    parser.add_argument("--real-val-per-class", type=int, default=5)
    parser.add_argument("--real-test-per-class", type=int, default=0)
    parser.add_argument("--support-repeat-factor", type=int, default=16)
    parser.add_argument("--synthetic-train-ratio", type=float, default=0.9)
    parser.add_argument("--synthetic-epochs", type=int, default=12)
    parser.add_argument("--synthetic-lr", type=float, default=1e-3)
    parser.add_argument("--real-epochs", type=int, default=20)
    parser.add_argument("--real-lr", type=float, default=3e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--logit-scale", type=float, default=20.0)
    parser.add_argument("--align-weight", type=float, default=0.5)
    parser.add_argument("--num-surface-points", type=int, default=4096)
    parser.add_argument("--axis-bins", type=int, default=12)
    parser.add_argument("--radial-bins", type=int, default=12)
    parser.add_argument("--pairwise-bins", type=int, default=12)
    parser.add_argument("--strong-synthetic-aug", action="store_true")
    args = parser.parse_args()

    if not 0.0 <= args.explicit_test_val_ratio < 1.0:
        raise ValueError("--explicit-test-val-ratio must be in [0, 1).")

    set_seed(args.seed)
    os.makedirs(args.output_dir, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    canonical_classes = sorted(discover_assets(args.asset_dir).keys())
    print(f"Discovered geometry classes: {canonical_classes}")

    geometry_prototypes = build_geometry_prototypes(
        asset_dir=args.asset_dir,
        canonical_classes=canonical_classes,
        num_surface_points=args.num_surface_points,
        axis_bins=args.axis_bins,
        radial_bins=args.radial_bins,
        pairwise_bins=args.pairwise_bins,
        device=device,
    )
    geometry_dim = int(geometry_prototypes.shape[1])
    print(f"Geometry prototype dimension: {geometry_dim}")

    support_samples, real_val_samples, real_test_samples, real_val_source = load_real_splits(args, canonical_classes)

    synthetic_train_samples: List[Tuple[str, int]] = []
    synthetic_val_samples: List[Tuple[str, int]] = []
    if args.synthetic_dir:
        if not os.path.isdir(args.synthetic_dir):
            raise FileNotFoundError(f"Synthetic data directory not found: {args.synthetic_dir}")
        synthetic_base = datasets.ImageFolder(args.synthetic_dir)
        if list(synthetic_base.classes) != canonical_classes:
            raise ValueError(
                "Synthetic dataset classes do not match geometry classes.\n"
                f"Synthetic classes: {synthetic_base.classes}\n"
                f"Geometry classes: {canonical_classes}"
            )
        synthetic_train_samples, synthetic_val_samples = split_samples_per_class(
            synthetic_base, train_ratio=args.synthetic_train_ratio, seed=args.seed
        )

    train_tf, eval_tf = build_transforms(args.image_size, strong_aug=args.strong_synthetic_aug)
    synthetic_train = SubsetImageFolder(synthetic_train_samples, transform=train_tf) if synthetic_train_samples else None
    synthetic_val = SubsetImageFolder(synthetic_val_samples, transform=eval_tf) if synthetic_val_samples else None
    real_support = SubsetImageFolder(support_samples, transform=train_tf, repeat_factor=args.support_repeat_factor)
    real_val = SubsetImageFolder(real_val_samples, transform=eval_tf)
    real_test = SubsetImageFolder(real_test_samples, transform=eval_tf)

    synthetic_train_loader = (
        make_loader(synthetic_train, args.batch_size, shuffle=True, num_workers=args.num_workers)
        if synthetic_train is not None
        else None
    )
    synthetic_val_loader = (
        make_loader(synthetic_val, args.batch_size, shuffle=False, num_workers=args.num_workers)
        if synthetic_val is not None
        else None
    )
    real_support_loader = make_loader(
        real_support, min(args.batch_size, len(real_support)), shuffle=True, num_workers=args.num_workers
    )
    real_val_loader = make_loader(real_val, min(args.batch_size, len(real_val)), shuffle=False, num_workers=args.num_workers)
    real_test_loader = make_loader(real_test, min(args.batch_size, len(real_test)), shuffle=False, num_workers=args.num_workers)

    print("\nDataset summary")
    print(f"Classes: {canonical_classes}")
    print(f"Real support samples: {len(support_samples)}")
    print(f"Real val samples: {len(real_val_samples)} ({real_val_source})")
    print(f"Real test samples: {len(real_test_samples)}")
    print(f"Synthetic train samples: {len(synthetic_train_samples)}")
    print(f"Synthetic val samples: {len(synthetic_val_samples)}")

    model = ImageToGeometryModel(
        backbone=args.backbone,
        geometry_dim=geometry_dim,
        projector_hidden_dim=args.projector_hidden_dim,
        dropout=args.projector_dropout,
    ).to(device)
    configure_trainable_params(model, args.train_mode)

    if synthetic_train_loader is not None and synthetic_val_loader is not None:
        print("\nStage 1: synthetic image -> geometry alignment")
        model, _ = fit_stage(
            stage_name="stage1_synthetic",
            model=model,
            train_loader=synthetic_train_loader,
            val_loader=synthetic_val_loader,
            geometry_prototypes=geometry_prototypes,
            epochs=args.synthetic_epochs,
            lr=args.synthetic_lr,
            weight_decay=args.weight_decay,
            device=device,
            logit_scale=args.logit_scale,
            align_weight=args.align_weight,
        )
        stage1_path = os.path.join(args.output_dir, "stage1_synthetic_best.pth")
        torch.save(model.state_dict(), stage1_path)
        print(f"Saved stage 1 model to {stage1_path}")

    print("\nStage 2: real few-shot calibration into geometry space")
    model, _ = fit_stage(
        stage_name="stage2_real",
        model=model,
        train_loader=real_support_loader,
        val_loader=real_val_loader,
        geometry_prototypes=geometry_prototypes,
        epochs=args.real_epochs,
        lr=args.real_lr,
        weight_decay=args.weight_decay,
        device=device,
        logit_scale=args.logit_scale,
        align_weight=args.align_weight,
    )

    final_model_path = os.path.join(args.output_dir, "image_to_geometry_best.pth")
    torch.save(model.state_dict(), final_model_path)
    print(f"Saved final model to {final_model_path}")

    val_loss, val_acc, _, _, _, _ = evaluate_full(
        model, real_val_loader, geometry_prototypes, device, args.logit_scale, args.align_weight
    )
    test_loss, test_acc, _, _, report, cm = evaluate_full(
        model, real_test_loader, geometry_prototypes, device, args.logit_scale, args.align_weight
    )

    print("\nFinal results")
    print(f"Validation accuracy: {val_acc:.4f}")
    print(f"Test accuracy: {test_acc:.4f}")

    summary = RunSummary(
        classes=canonical_classes,
        geometry_dim=geometry_dim,
        backbone=args.backbone,
        train_mode=args.train_mode,
        synthetic_stage_enabled=bool(synthetic_train_samples),
        synthetic_train_count=len(synthetic_train_samples),
        synthetic_val_count=len(synthetic_val_samples),
        real_support_count=len(support_samples),
        real_val_count=len(real_val_samples),
        real_test_count=len(real_test_samples),
        val_accuracy=val_acc,
        test_accuracy=test_acc,
        val_loss=val_loss,
        test_loss=test_loss,
        confusion_matrix=cm,
        classification_report=report,
        model_path=final_model_path,
    )

    summary_path = os.path.join(args.output_dir, "image_to_geometry_summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(asdict(summary), f, indent=2)
    print(f"Saved summary to {summary_path}")


if __name__ == "__main__":
    main()
