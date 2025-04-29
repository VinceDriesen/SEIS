---------------------------- MODULE aisleChecker ----------------------------

EXTENDS Integers, Sequences, TLC

CONSTANTS RobotIds, AisleWidth, MaxRobots

VARIABLES robotsInAisle, robotsPositions

TypeInvariant == 
    /\ robotsInAisle \subseteq RobotIds
    /\ robotsPositions \in [RobotIds -> 0..AisleWidth]

MutualExclusion == robotsInAisle = {} 
                                \/ (\E r \in robotsInAisle : robotsInAisle = {r})

EnterAisle(r) ==
    /\ r \notin robotsInAisle
    /\ robotsPositions[r] = 0
    /\ robotsInAisle = {}
    /\ robotsInAisle' = robotsInAisle \union {r} 
    /\ robotsPositions' = [robotsPositions EXCEPT ![r] = 1]

MoveForward(r) ==
    /\ r \in robotsInAisle
    /\ robotsPositions[r] < AisleWidth
    /\ robotsPositions' = [robotsPositions EXCEPT ![r] = robotsPositions[r] + 1]
    /\ UNCHANGED robotsInAisle

ExitAisle(r) == 
    /\ r \in robotsInAisle
    /\ robotsPositions[r] = AisleWidth
    /\ robotsInAisle' = robotsInAisle \ {r}
    /\ robotsPositions' = [robotsPositions EXCEPT ![r] = 0]

Next == 
    \E r \in RobotIds : (EnterAisle(r) \/ MoveForward(r) \/ ExitAisle(r))

Init ==
    /\ robotsInAisle = {}
    /\ robotsPositions = [r \in RobotIds |-> 0]

Spec == Init /\ [][Next]_<<robotsInAisle, robotsPositions>>

THEOREM Spec => []MutualExclusion
=============================================================================