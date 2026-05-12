# Lynxmotion-SES-Pro-550mm-6DOF-Robot-Arm

Lynxmotion 550 mm 6DOF Robot Arm
1.	Joint limits (deg.):
a.	J1: [-180, +180]
b.	J2: [-90, +90]
c.	J3: [-115, +115]
Though J2 and J3 have different limits, but actually their limits are coupled with each other in the sense that the instantaneous limit of J2 or J3 joint must be computed as per the current position of the other one.
d.	J4: [-130, +160]
Cannot go to -180 and +180 deg. because of the excessive tension appearing in the connection wires.
e.	J5: [-180, +105]
Cannot go to +180 deg. because of the excessive tension appearing in the connection wires.
f.	J6
Cannot test since we don’t have any grippers yet.

2.	CAD models available at https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-pro/ses-pro-arms/ses-pro-550-6-dof-arm/#HCADFiles
3.	ad
