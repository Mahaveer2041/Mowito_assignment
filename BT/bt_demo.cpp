#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

using namespace BT;

// Custom action nodes
class MoveTowardsRoomDoor : public SyncActionNode
{
public:
    MoveTowardsRoomDoor(const std::string& name) : SyncActionNode(name, {})
    {}

    NodeStatus tick() override
    {
        std::cout << "Moving towards the room door..." << std::endl;
        return NodeStatus::SUCCESS;
    }
};

class IsDoorClosed : public ConditionNode
{
public:
    IsDoorClosed(const std::string& name) : ConditionNode(name, {})
    {}

    NodeStatus tick() override
    {
        // In a real implementation, this would check the door status
        // For this example, we'll randomly return yes/no
        bool is_closed = (rand() % 2) == 0;
        std::cout << "Is the room door closed? " << (is_closed ? "Yes" : "No") << std::endl;
        return is_closed ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class OpenDoor : public SyncActionNode
{
public:
    OpenDoor(const std::string& name) : SyncActionNode(name, {})
    {}

    NodeStatus tick() override
    {
        std::cout << "Opening the door..." << std::endl;
        return NodeStatus::SUCCESS;
    }
};

class EnterRoom : public SyncActionNode
{
public:
    EnterRoom(const std::string& name) : SyncActionNode(name, {})
    {}

    NodeStatus tick() override
    {
        std::cout << "Entering the room..." << std::endl;
        return NodeStatus::SUCCESS;
    }
};

class MoveTowardsFridgeDoor : public SyncActionNode
{
public:
    MoveTowardsFridgeDoor(const std::string& name) : SyncActionNode(name, {})
    {}

    NodeStatus tick() override
    {
        std::cout << "Moving towards the fridge door..." << std::endl;
        return NodeStatus::SUCCESS;
    }
};

class IsFridgeDoorClosed : public ConditionNode
{
public:
    IsFridgeDoorClosed(const std::string& name) : ConditionNode(name, {})
    {}

    NodeStatus tick() override
    {
        // In a real implementation, this would check the fridge door status
        bool is_closed = (rand() % 2) == 0;
        std::cout << "Is the fridge door closed? " << (is_closed ? "Yes" : "No") << std::endl;
        return is_closed ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

class FindApple : public SyncActionNode
{
public:
    FindApple(const std::string& name) : SyncActionNode(name, {})
    {}

    NodeStatus tick() override
    {
        std::cout << "Finding the apple..." << std::endl;
        return NodeStatus::SUCCESS;
    }
};

class PickApple : public SyncActionNode
{
public:
    PickApple(const std::string& name) : SyncActionNode(name, {})
    {}

    NodeStatus tick() override
    {
        std::cout << "Picking the apple..." << std::endl;
        return NodeStatus::SUCCESS;
    }
};

class CloseFridgeDoor : public SyncActionNode
{
public:
    CloseFridgeDoor(const std::string& name) : SyncActionNode(name, {})
    {}

    NodeStatus tick() override
    {
        std::cout << "Closing the fridge door..." << std::endl;
        return NodeStatus::SUCCESS;
    }
};

class ExitRoom : public SyncActionNode
{
public:
    ExitRoom(const std::string& name) : SyncActionNode(name, {})
    {}

    NodeStatus tick() override
    {
        std::cout << "Exiting the room..." << std::endl;
        return NodeStatus::SUCCESS;
    }
};

// Function to register all custom nodes
void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<MoveTowardsRoomDoor>("MoveTowardsRoomDoor");
    factory.registerNodeType<IsDoorClosed>("IsDoorClosed");
    factory.registerNodeType<OpenDoor>("OpenDoor");
    factory.registerNodeType<EnterRoom>("EnterRoom");
    factory.registerNodeType<MoveTowardsFridgeDoor>("MoveTowardsFridgeDoor");
    factory.registerNodeType<IsFridgeDoorClosed>("IsFridgeDoorClosed");
    factory.registerNodeType<FindApple>("FindApple");
    factory.registerNodeType<PickApple>("PickApple");
    factory.registerNodeType<CloseFridgeDoor>("CloseFridgeDoor");
    factory.registerNodeType<ExitRoom>("ExitRoom");
}

int main()
{
    BehaviorTreeFactory factory;
    RegisterNodes(factory);
    
    // Define the behavior tree
    std::string tree_xml = R"(
    <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
            <Sequence>
                <MoveTowardsRoomDoor/>
                <Fallback>
                    <IsDoorClosed/>
                    <OpenDoor/>
                </Fallback>
                <EnterRoom/>
                <MoveTowardsFridgeDoor/>
                <Fallback>
                    <IsFridgeDoorClosed/>
                    <OpenDoor/>
                </Fallback>
                <FindApple/>
                <PickApple/>
                <CloseFridgeDoor/>
                <MoveTowardsRoomDoor/>
                <ExitRoom/>
            </Sequence>
        </BehaviorTree>
    </root>
    )";
    
    auto tree = factory.createTreeFromText(tree_xml);
    tree.tickWhileRunning();
    
    return 0;
}