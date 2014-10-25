(* ::Package:: *)

ClearAll["Global`*"];
<<Quaternions`;

(* Functions *)
jacobian[ff_, dd_]:=D[Flatten[ff], {Flatten[dd]}]

rotateVectorByQuaternion[{x_, y_, z_}, q_] :=
  Drop[List @@ (q ** Quaternion[0, x, y, z] ** Conjugate[q]), 1]

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
 * Small angle approximation - suitable for covariance propagation.
 * Note that state update equations shall use the full conversion algorithm anyway.
 *)
simplifiedQuaternionFromEuler[ai_, aj_, ak_] := Quaternion[1, ai/2, aj/2, ak/2]

simplifiedDeltaQuaternionFromAngularRate[{wx_,wy_,wz_},dt_] :=
  simplifiedQuaternionFromEuler[wx dt, wy dt, wz dt]

deltaQuaternionFromAngularRate[{wx_,wy_,wz_},dt_] := quaternionFromEuler[wx dt, wy dt, wz dt]



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

bodyAngVel = {wlx,wly,wlz};

bodyGyroBias = {bwlx, bwly, bwlz};

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
Print["x=", x//MatrixForm]
Print["f=", f//MatrixForm]

F = jacobian[f,x];

Print["F=", F//MatrixForm]

(* Accelerometer update *)
himuacc1 = rotateVectorByQuaternion[gravity1,q];
Himuacc1 = jacobian[himuacc1,x];

Print["himuacc1=", himuacc1//MatrixForm]
Print["Himuacc1=", Himuacc1//MatrixForm]

(* Gyro update *)
hgyro = bodyAngVel + bodyGyroBias;
Hgyro = jacobian[hgyro,x];

Print["hgyro=", hgyro//MatrixForm]
Print["Hgyro=", Hgyro//MatrixForm]

