---------------------------- MODULE aisleChecker ----------------------------

EXTENDS Integers, Sequences, TLC

CONSTANTS NumberOfAisles, AisleWidth, MaxRobots

VARIABLES robotsInAisle, robotsPositions

ASSUME NumberOfAisles \in Nat \ {0}
ASSUME AisleWidth \in Nat \ {0}
ASSUME MaxRobots \in Nat \ {0}

RobotIds == 1..MaxRobots
Aisles == 1..NumberOfAisles
  

TypeInvariant == 
    /\ robotsInAisle \in [Aisles -> SUBSET RobotIds]
    /\ \A a \in Aisles : robotsInAisle[a] \subseteq RobotIds
    /\ robotsPositions \in [RobotIds -> [Aisles -> 0..AisleWidth]]
    /\ \A r \in RobotIds : DOMAIN robotsPositions[r] = Aisles
    
    
AisleMutualExclusion == \A a \in Aisles : (robotsInAisle[a] = {}) \/ (\E r \in robotsInAisle[a] : robotsInAisle[a] = {r})

EnterAisle(r, a) ==
    /\ r \in RobotIds
    /\ a \in Aisles
    /\ robotsInAisle[a] = {}
    /\ r \notin robotsInAisle[a]
    /\ robotsPositions[r][a] = 0
    /\ \A b \in Aisles \ {a} : r \notin robotsInAisle[b]
    /\ robotsInAisle' = [robotsInAisle EXCEPT ![a] = robotsInAisle[a] \union {r}]
    /\ robotsPositions' = [robotsPositions EXCEPT ![r] = [b \in Aisles |-> IF b = a THEN 1 ELSE robotsPositions[r][b]]]

MoveForward(r, a) ==
    /\ r \in RobotIds
    /\ a \in Aisles
    /\ r \in robotsInAisle[a]
    /\ robotsPositions[r][a] < AisleWidth
    /\ robotsPositions' = [robotsPositions EXCEPT ![r] = [b \in Aisles |-> IF b = a THEN robotsPositions[r][a] + 1 ELSE robotsPositions[r][b]]]
    /\ UNCHANGED robotsInAisle

ExitAisle(r, a) ==
    /\ r \in RobotIds
    /\ a \in Aisles
    /\ r \in robotsInAisle[a]
    /\ robotsPositions[r][a] = AisleWidth
    /\ robotsInAisle' = [robotsInAisle EXCEPT ![a] = robotsInAisle[a] \ {r}]
    /\ robotsPositions' = [robotsPositions EXCEPT ![r] = [b \in Aisles |-> IF b = a THEN 0 ELSE robotsPositions[r][b]]]

Next ==
    \E r \in RobotIds, a \in Aisles :
        (EnterAisle(r, a) \/ MoveForward(r, a) \/ ExitAisle(r, a))

Init ==
    /\ robotsInAisle = [a \in Aisles |-> {}]
    /\ robotsPositions = [r \in RobotIds |-> [a \in Aisles |-> 0]]

Spec == Init /\ [][Next]_<<robotsInAisle, robotsPositions>>

THEOREM Spec => []AisleMutualExclusion
=============================================================================