(* ::Package:: *)

ClearAll["Global`*"];
<<Quaternions`;

(* Functions *)
jacobian[ff_, dd_]:=D[Flatten[ff], {Flatten[dd]}]

rotateVectorByQuaternion[{{x_}, {y_}, {z_}}, q_] :=
  {Drop[List @@ (q ** Quaternion[0, x, y, z] ** Conjugate[q]), 1]}\[Transpose]

quaternionFromEuler[ai_, aj_, ak_] := With[{
    ci = Cos[ai/2],
    si = Sin[ai/2],
    cj = Cos[aj/2],
    sj = Sin[aj/2],
    ck = Cos[ak/2],
    sk = Sin[ak/2]},
   Module[{
     cc = ci ck,
     cs = ci sk,
     sc = si ck,
     ss = si sk},
    Quaternion[
     cj cc + sj ss,
     cj sc - sj cs,
     cj ss + sj cc,
     cj cs - sj sc]]]

(*
 * Ref. "Euler Angles, Quaternions, and Transformation Matrices" - NASA (Shuttle program)
 * Ref. http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 *)
eulerFromQuaternion[q_] :=
 With[{q0 = q[[1]], q1 = q[[2]], q2 = q[[3]], q3 = q[[4]]}, {
   {ArcTan[1 - 2 (q1^2 + q2^2), 2 (q0 q1 + q2 q3)]},
   {ArcSin[2 (q0 q2 - q3 q1)]},
   {ArcTan[1 - 2 (q2^2 + q3^2), 2 (q0 q3 + q1 q2)]}}]

(*
 * Small angle approximation - suitable for covariance propagation.
 * Note that state update equations shall use the full conversion algorithm anyway.
 *)
simplifiedQuaternionFromEuler[ai_, aj_, ak_] := Quaternion[1, ai/2, aj/2, ak/2]

simplifiedDeltaQuaternionFromAngularRate[{{wx_},{wy_},{wz_}},dt_] :=
  simplifiedQuaternionFromEuler[wx dt, wy dt, wz dt]

deltaQuaternionFromAngularRate[{{wx_},{wy_},{wz_}},dt_] := quaternionFromEuler[wx dt, wy dt, wz dt]



gravity1 = {{0},{0},{1}};

(* State vector *)
x = {
 {qw},
 {qx},
 {qy},
 {qz},
 {wlx},
 {wly},
 {wlz},
 {bwlx},
 {bwly},
 {bwlz}
};
q = Quaternion[qw, qx, qy, qz];
qr = Conjugate[q];

bodyAngVel = {{wlx},{wly},{wlz}};

bodyGyroBias = {{bwlx}, {bwly}, {bwlz}};

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
 {bwlx},
 {bwly},
 {bwlz}
};

F = jacobian[f,x];

(* Accelerometer update *)
himuacc1 = rotateVectorByQuaternion[gravity1,q];
Himuacc1 = jacobian[himuacc1,x];

(* Gyro update *)
hgyro = bodyAngVel + bodyGyroBias;
Hgyro = jacobian[hgyro,x];

(*
 * Quaternion covariance to Euler covariance
 * Ref. "Development of a Real-Time Attitude System Using a Quaternion
 * Parameterization and Non-Dedicated GPS Receivers" - John B. Schleppe (page 69)
 *)
rpy = eulerFromQuaternion[q];
QuaternionToEulerCovarianceJacobian = jacobian[rpy, List@@q];



(* Outputs *)
Print["x=", x//MatrixForm]
Print["f=", f//MatrixForm]
Print["F=", F//MatrixForm]
Print["himuacc1=", himuacc1//MatrixForm]
Print["Himuacc1=", Himuacc1//MatrixForm]
Print["hgyro=", hgyro//MatrixForm]
Print["Hgyro=", Hgyro//MatrixForm]
Print["rpy=", rpy//MatrixForm]
Print["QuaternionToEulerCovarianceJacobian=", QuaternionToEulerCovarianceJacobian//MatrixForm]

