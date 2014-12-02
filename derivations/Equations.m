(* ::Package:: *)

ClearAll["Global`*"];
ClearAll["X`*"];
ClearAll["H`*"];
$ContextPath = Prepend[$ContextPath, "X`"];
$ContextPath = Prepend[$ContextPath, "H`"];
$ContextPath = $ContextPath // DeleteDuplicates
<<NavFunctions`;
<<Quaternions`;
<<CPPCodeGeneration`;


StdGravity = 9.80665;
gravity = {{0}, {0}, {StdGravity}};

(*
 * State vector
 *)
x = Join[
 defineSymbolicColumnVectorXYZ["X`pwi"], (* 1, position *)
 defineSymbolicColumnVectorXYZ["X`vwi"], (* 4, velocity *)
 quaternionAsColumnVector[defineSymbolicQuaternion["X`qwi"]], (* 7, world --> IMU rotation *)
 defineSymbolicColumnVectorXYZ["X`a"], (* 11, acceleration *)
 defineSymbolicColumnVectorXYZ["X`w"], (* 14, angular velocity *)
 defineSymbolicColumnVectorXYZ["X`ja"], (* 17, linear jerk *)
 defineSymbolicColumnVectorXYZ["X`jw"], (* 20, angular acceleration *)
 defineSymbolicColumnVectorXYZ["X`ba"], (* 23, accelerometer bias *)
 defineSymbolicColumnVectorXYZ["X`bw"], (* 26, rate gyro bias *)
 defineSymbolicColumnVectorXYZ["X`pvw"],(* 29, visual --> world translation *)
 quaternionAsColumnVector[defineSymbolicQuaternion["X`qvw"]]]; (* 32, visual --> world rotation *)

printMatrixByName["x"]

(*
 * Time update: f(x, dt)
 *)
f = x;
f[[1;;16]] = Join[
 pwi + dt vwi + dt dt rotateVectorByQuaternion[a + dt ja, Conjugate[qwi]],
 vwi + dt rotateVectorByQuaternion[a + dt ja, Conjugate[qwi]],
 quaternionAsColumnVector[qwi ** simplifiedDeltaQuaternionFromAngularRate[w + dt jw, dt]],
 a + dt ja,
 w + dt jw];
F = jacobian[f,x];
f[[7;;10]] = quaternionAsColumnVector[qwi ** deltaQuaternionFromAngularRate[w + dt jw, dt]];

printMatrixByName /@ {"f", "F"};

(*
 * State vector normalization
 *)
normx = x;
normx[[7;;10]] = quaternionAsColumnVector[normalizeQuaternion[qwi]];
normx[[32;;35]] = quaternionAsColumnVector[normalizeQuaternion[qvw]];

printMatrixByName @ "normx";

(*
 * Min Q, initial P
 * Q is the speed of covariance growth, units per second
 *)
Qmindiag = ConstantArray[0,Length[x]];
Qmindiag[[1;;3]] = ConstantArray[1, 3]; (* pos *)
Qmindiag[[4;;6]] = ConstantArray[1, 3]; (* vel *)
Qmindiag[[7;;10]] = ConstantArray[0.1, 4]; (* att *)
Qmindiag[[11;;16]] = ConstantArray[100, 6]; (* a w *)
Qmindiag[[17;;22]] = ConstantArray[1000, 6]; (* jerks *)
(* Accel/gyro biases don't drift over time *)
Qmindiag[[29;;35]] = ConstantArray[100, 7]; (* visual frame offsets *)

Pinitdiag = ConstantArray[0.01,Length[x]];
Pinitdiag[[11;;22]] = ConstantArray[100,12];
Pinitdiag[[23;;28]] = ConstantArray[10^-6,6];
Pinitdiag[[29;;35]] = ConstantArray[10000,7];

printMatrixByName /@ {"Qmindiag", "Pinitdiag"};

(*
 * Measurement update equations: h(x)
 *)
makeMeasurementPrediction[name_, equation_] := {
  Set[Evaluate[Symbol["H`h" <> ToString[name]]], equation],
  Set[Evaluate[Symbol["H`H" <> ToString[name]]], jacobian[equation, x]]}

makeMeasurementPrediction["acc", a + ba + rotateVectorByQuaternion[gravity, qwi]];
makeMeasurementPrediction["gyro", w + bw];

makeMeasurementPrediction["gnsspos", pwi];
makeMeasurementPrediction["gnssvel", vwi];

makeMeasurementPrediction["vispos", pwi + rotateVectorByQuaternion[pvw, Conjugate[qvw]]];
makeMeasurementPrediction["visvel", rotateVectorByQuaternion[vwi, Conjugate[qwi]]];
makeMeasurementPrediction["visatt", eulerFromQuaternion[qwi ** qvw]];

makeMeasurementPrediction["climbrate", {vwi[[3]]}];

printMatrixByName /@ Names["H`*"];


(*
 * Quaternion covariance to Euler covariance
 * Ref. "Development of a Real-Time Attitude System Using a Quaternion
 * Parameterization and Non-Dedicated GPS Receivers" - John B. Schleppe (page 69)
 *)
eulerFromQuaternionJacobian = jacobian[eulerFromQuaternion[qwi], List @@ qwi];
quaternionFromEulerJacobian = jacobian[quaternionFromEuler[roll, pitch, yaw], {roll, pitch, yaw}];
printMatrixByName["eulerFromQuaternionJacobian"];
printMatrixByName["quaternionFromEulerJacobian"];

(*
 * GNSS velocity conversion
 *)
gnssVelocityLonLatClimbJacobian =
 jacobian[gnssVelocityLonLatClimb[gnssTrack, gnssSpeed, gnssClimb], {gnssTrack, gnssSpeed, gnssClimb}];
printMatrixByName["gnssVelocityLonLatClimbJacobian"];



srcdir = FileNameJoin[{NotebookDirectory[], "..", "src"}];
expandTemplateFiles[srcdir, {"*.cpp", "*.cc", "*.hpp", "*.h"}]















