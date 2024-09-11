#include "RoadPathfindingComponent.h"
#include "Containers/Queue.h"
#include "Algo/Reverse.h"

URoadPathfindingComponent::URoadPathfindingComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
}

TArray<TSharedPtr<FPathNode>> URoadPathfindingComponent::FindAllNodes(const TArray<USplineComponent*>& SplineComponents)
{
    TMap<FVector, TSharedPtr<FPathNode>> NodeMap; // Use smart pointers for NodeMap to manage neighbors correctly

    // Create nodes and link neighbors
    for (USplineComponent* Spline : SplineComponents)
    {
        FVector StartLocation = Spline->GetLocationAtSplinePoint(0, ESplineCoordinateSpace::World);
        FVector EndLocation = Spline->GetLocationAtSplinePoint(Spline->GetNumberOfSplinePoints() - 1, ESplineCoordinateSpace::World);

        // Check or add StartNode
        TSharedPtr<FPathNode>* StartNodePtr = NodeMap.Find(StartLocation);
        if (StartNodePtr == nullptr)
        {
            TSharedPtr<FPathNode> NewNode = MakeShared<FPathNode>(StartLocation);
            NodeMap.Add(StartLocation, NewNode);
            StartNodePtr = &NewNode;
        }

        // Check or add EndNode
        TSharedPtr<FPathNode>* EndNodePtr = NodeMap.Find(EndLocation);
        if (EndNodePtr == nullptr)
        {
            TSharedPtr<FPathNode> NewNode = MakeShared<FPathNode>(EndLocation);
            NodeMap.Add(EndLocation, NewNode);
            EndNodePtr = &NewNode;
        }

        // Link nodes as neighbors if they are different
        if (*StartNodePtr != *EndNodePtr)
        {
            if (!(*StartNodePtr)->Neighbors.Contains(*EndNodePtr))
            {
                (*StartNodePtr)->Neighbors.Add(*EndNodePtr);
            }

            if (!(*EndNodePtr)->Neighbors.Contains(*StartNodePtr))
            {
                (*EndNodePtr)->Neighbors.Add(*StartNodePtr);
            }
        }
    }

    // Convert NodeMap values to TArray
    TArray<TSharedPtr<FPathNode>> PathNodes;
    NodeMap.GenerateValueArray(PathNodes);

    return PathNodes;
}

TArray<TSharedPtr<FPathNode>> URoadPathfindingComponent::AStarPathfinding(TSharedPtr<FPathNode> StartNode, TSharedPtr<FPathNode> GoalNode, const TArray<TSharedPtr<FPathNode>>& AllNodes)
{
    if (!StartNode.IsValid() || !GoalNode.IsValid())
    {
        return TArray<TSharedPtr<FPathNode>>();
    }

    TMap<FVector, TSharedPtr<FPathNode>> NodeMap;
    for (TSharedPtr<FPathNode> Node : AllNodes)
    {
        NodeMap.Add(Node->Location, Node);
    }

    // Open set (nodes to be evaluated) and closed set (nodes already evaluated)
    TSet<TSharedPtr<FPathNode>> OpenSet;
    TSet<TSharedPtr<FPathNode>> ClosedSet;
    TMap<TSharedPtr<FPathNode>, TSharedPtr<FPathNode>> CameFrom; // For path reconstruction
    TMap<TSharedPtr<FPathNode>, float> GScore; // Cost from start node
    TMap<TSharedPtr<FPathNode>, float> FScore; // Total cost (GScore + heuristic)

    // Initialize scores
    OpenSet.Add(StartNode);
    GScore.Add(StartNode, 0.0f);
    FScore.Add(StartNode, FVector::Distance(StartNode->Location, GoalNode->Location));

    // Heuristic function (Euclidean distance)
    auto Heuristic = [](const FVector& A, const FVector& B) {
        return FVector::Distance(A, B);
        };

    while (OpenSet.Num() > 0)
    {
        // Get node with the lowest FScore
        TSharedPtr<FPathNode> CurrentNode = nullptr;
        float LowestScore = FLT_MAX;
        for (TSharedPtr<FPathNode> Node : OpenSet)
        {
            float Score = FScore[Node];
            if (Score < LowestScore)
            {
                LowestScore = Score;
                CurrentNode = Node;
            }
        }

        if (CurrentNode == GoalNode)
        {
            // Reconstruct path
            TArray<TSharedPtr<FPathNode>> Path;
            while (CameFrom.Contains(CurrentNode))
            {
                Path.Add(CurrentNode);
                CurrentNode = CameFrom[CurrentNode];
            }
            Path.Add(StartNode);
            Algo::Reverse(Path);
            return Path;
        }

        OpenSet.Remove(CurrentNode);
        ClosedSet.Add(CurrentNode);

        for (TSharedPtr<FPathNode> Neighbor : CurrentNode->Neighbors)
        {
            TSharedPtr<FPathNode>* ActualNeighborPtr = NodeMap.Find(Neighbor->Location);
            if (ActualNeighborPtr == nullptr || ClosedSet.Contains(*ActualNeighborPtr))
            {
                continue;
            }

            TSharedPtr<FPathNode> ActualNeighbor = *ActualNeighborPtr;

            float TentativeGScore = GScore[CurrentNode] + FVector::Distance(CurrentNode->Location, ActualNeighbor->Location);

            if (!OpenSet.Contains(ActualNeighbor))
            {
                OpenSet.Add(ActualNeighbor);
            }
            else if (TentativeGScore >= GScore[ActualNeighbor])
            {
                continue;
            }

            CameFrom.Add(ActualNeighbor, CurrentNode);
            GScore.Add(ActualNeighbor, TentativeGScore);
            FScore.Add(ActualNeighbor, TentativeGScore + Heuristic(ActualNeighbor->Location, GoalNode->Location));
        }
    }

    // No path found
    return TArray<TSharedPtr<FPathNode>>();
}

TSharedPtr<FPathNode> URoadPathfindingComponent::FindNearestNodeByLocation(const FVector& Location, const TArray<TSharedPtr<FPathNode>>& AllNodes)
{
    if (AllNodes.Num() == 0)
    {
        return nullptr;
    }

    TSharedPtr<FPathNode> NearestNode = nullptr;
    float MinDistance = FLT_MAX;

    for (TSharedPtr<FPathNode> Node : AllNodes)
    {
        float Distance = FVector::Dist(Location, Node->Location);
        if (Distance < MinDistance)
        {
            MinDistance = Distance;
            NearestNode = Node;
        }
    }

    return NearestNode;
}

TArray<FVector> URoadPathfindingComponent::GetLocationsFromPathNodes(const TArray<TSharedPtr<FPathNode>>& PathNodes)
{
    TArray<FVector> Locations;

    // Iterate through the path nodes and collect their locations
    for (TSharedPtr<FPathNode> Node : PathNodes)
    {
        if (Node.IsValid())
        {
            Locations.Add(Node->Location);
        }
    }

    return Locations;
}
