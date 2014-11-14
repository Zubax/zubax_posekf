(* ::Package:: *)

<<Quaternions`;

rotateVectorByQuaternion[{{x_}, {y_}, {z_}}, q_] :=
  {Drop[List @@ (q ** Quaternion[0, x, y, z] ** Conjugate[q]), 1]}\[Transpose]

quaternionAsColumnVector[q_] := {List @@ q}\[Transpose]

jacobian[ff_List, dd_] := D[Flatten[ff], {Flatten[dd]}]
jacobian[ff_Quaternion, dd_] := D[List @@ ff, {Flatten[dd]}]

(*
 * Ref. ROS Python TF API module
 *)
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

(*
 * GNSS velocity vector from track, horizontal speed and climb rate.
 *)
gnssVelocityLonLatClimb[trackRad_, speedMS_, climbMS_] := {
  {Sin[trackRad] speedMS}, (* Lon *)
  {Cos[trackRad] speedMS}, (* Lat *)
  {climbMS}}

printMatrixByName[name_] := Print[name, "=", MatrixForm[Symbol[name]]]

(*
 * Symbolic helpers
 *)
symbolicVector[name_, components_] := Table[Symbol[ToString[name] <> ToString[m]], {m, components}]

symbolicQuaternion[name_] := Quaternion @@ symbolicVector[ToString[name], {"w", "x", "y", "z"}]

defineSymbolicColumnVectorXYZ[sym_] :=
 Set[Evaluate[Symbol[ToString[sym]]], {symbolicVector[ToString[sym], {"x", "y", "z"}]}\[Transpose]]

defineSymbolicQuaternion[sym_] := Set[Evaluate[Symbol[ToString[sym]]], symbolicQuaternion[ToString[sym]]]
