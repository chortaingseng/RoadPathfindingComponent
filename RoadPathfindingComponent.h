#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/SplineComponent.h"
#include "RoadPathfindingComponent.generated.h"

USTRUCT()
struct FPathNode {
    GENERATED_BODY()

    FVector Location;

    TArray<TSharedPtr<FPathNode>> Neighbors;

    FPathNode() : Location(FVector::ZeroVector) {}
    FPathNode(FVector InLocation) : Location(InLocation) {}

    bool operator==(const FPathNode& Other) const
    {
        return Location.Equals(Other.Location);
    }

    bool operator!=(const FPathNode& Other) const
    {
        return !(*this == Other);
    }
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class ROADNETWORKTOOL_API URoadPathfindingComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    URoadPathfindingComponent();

    TArray<TSharedPtr<FPathNode>> FindAllNodes(const TArray<USplineComponent*>& SplineComponents);

    TArray<TSharedPtr<FPathNode>> AStarPathfinding(TSharedPtr<FPathNode> StartNode, TSharedPtr<FPathNode> GoalNode, const TArray<TSharedPtr<FPathNode>>& AllNodes);

    TSharedPtr<FPathNode> FindNearestNodeByLocation(const FVector& Location, const TArray<TSharedPtr<FPathNode>>& AllNodes);

    TArray<FVector> GetLocationsFromPathNodes(const TArray<TSharedPtr<FPathNode>>& PathNodes);
};
