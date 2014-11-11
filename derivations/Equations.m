(* ::Package:: *)

ClearAll["Global`*"];
<<NavFunctions`;
<<Quaternions`;


StdGravity = 9.80665;
gravity = {{0}, {0}, {StdGravity}};

(* State vector: q wl al bw ba *)
x = {
 {qw},
 {qx},
 {qy},
 {qz},
 {wlx},
 {wly},
 {wlz},
 {alx},
 {aly},
 {alz},
 {bwx},
 {bwy},
 {bwz},
 {bax},
 {bay},
 {baz}
};

q = Quaternion[qw, qx, qy, qz];
qr = Conjugate[q];

bodyAngVel = {{wlx},{wly},{wlz}};
bodyAccel = {{alx}, {aly}, {alz}};

biasGyro = {{bwx}, {bwy}, {bwz}};
biasAccel = {{bax}, {bay}, {baz}};

(* Time update *)
(* TODO: use the approach proposed in "GPS aided INS solution for OpenPilot" *)
newq = q ** simplifiedDeltaQuaternionFromAngularRate[bodyAngVel, dtf];
f = {
 {newq[[1]]},
 {newq[[2]]},
 {newq[[3]]},
 {newq[[4]]},
 {wlx},
 {wly},
 {wlz},
 {alx},
 {aly},
 {alz},
 {bwx},
 {bwy},
 {bwz},
 {bax},
 {bay},
 {baz}
};

F = jacobian[f,x];

(* Accelerometer update *)
hacc = bodyAccel + biasAccel + rotateVectorByQuaternion[gravity, qr];
Hacc = jacobian[hacc,x];

(* GNSS acceleration update *)
hgnssacc = rotateVectorByQuaternion[bodyAccel, q];
Hgnssacc = jacobian[hgnssacc,x];

(* Gyro update *)
hgyro = bodyAngVel + biasGyro;
Hgyro = jacobian[hgyro,x];

(*
 * Quaternion covariance to Euler covariance
 * Ref. "Development of a Real-Time Attitude System Using a Quaternion
 * Parameterization and Non-Dedicated GPS Receivers" - John B. Schleppe (page 69)
 *)
eulerFromQuaternionJacobian = jacobian[eulerFromQuaternion[q], List@@q];

(*
 * GNSS velocity conversion
 *)
gnssVelocityLonLatClimbJacobian =
 jacobian[gnssVelocityLonLatClimb[gnssTrack, gnssSpeed, gnssClimb], {gnssTrack, gnssSpeed, gnssClimb}];


(* Outputs *)
Print["x=", x//MatrixForm]
Print["f=", f//MatrixForm]
Print["F=", F//MatrixForm]
Print["hacc=", hacc//MatrixForm]
Print["Hacc=", Hacc//MatrixForm]
Print["hgnssacc=", hgnssacc//MatrixForm]
Print["Hgnssacc=", Hgnssacc//MatrixForm]
Print["hgyro=", hgyro//MatrixForm]
Print["Hgyro=", Hgyro//MatrixForm]
Print["eulerFromQuaternionJacobian=", eulerFromQuaternionJacobian//MatrixForm]
Print["gnssVelocityLonLatClimbJacobian=", gnssVelocityLonLatClimbJacobian//MatrixForm]




