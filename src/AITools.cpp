//
// Created by tim on 14.06.19.
//

#include "AITools.h"

namespace aiTools{
    constexpr int distanceSnitchSeeker = 2;

    auto computeOptimalPath(const std::shared_ptr<const gameModel::Player> &player, const gameModel::Position &destination,
                            const std::shared_ptr<const gameModel::Environment> &env) -> std::vector<gameModel::Position> {
        if(player->position == destination) {
            return {};
        }

        auto expandFunction = [&env, &player](const aiTools::SearchNode<gameModel::Position> &node) {
            std::vector<aiTools::SearchNode<gameModel::Position>> path;
            path.reserve(8);
            auto parent = std::make_shared<aiTools::SearchNode<gameModel::Position>>(node);
            for (const auto &pos : env->getAllLegalCellsAround(node.state, env->team1->hasMember(player))) {
                path.emplace_back(pos, parent, node.pathCost + 1);
            }

            return path;
        };

        auto evalFunction = [&destination](const aiTools::SearchNode<gameModel::Position> &pos) {
            int g = pos.parent.has_value() ? pos.parent.value()->pathCost + 1 : 1;
            return g + gameController::getDistance(pos.state, destination);
        };

        aiTools::SearchNode<gameModel::Position> startNode(player->position, std::nullopt, 0);
        aiTools::SearchNode<gameModel::Position> destinationNode(destination, std::nullopt, 0);
        auto res = aiTools::aStarSearch(startNode, destinationNode, expandFunction, evalFunction);
        std::vector<gameModel::Position> ret;
        if (res.has_value()) {
            ret.reserve(res->pathCost + 1);
            ret.emplace_back(res->state);
            auto parent = res->parent;
            while (parent.has_value()) {
                ret.emplace_back((*parent)->state);
                parent = (*parent)->parent;
            }
        }

        return ret;
    }

    auto getNextFanTurn(const State &state, const communication::messages::broadcast::Next &next) ->
        const communication::messages::request::DeltaRequest {
        using namespace communication::messages;
        auto activeEntityId = next.getEntityId();
        auto mySide = gameLogic::conversions::idToSide(activeEntityId);
        std::optional<types::EntityId> passiveEntityId;
        if (activeEntityId == types::EntityId::LEFT_NIFFLER ||
            activeEntityId == types::EntityId::RIGHT_NIFFLER) {
            if(isNifflerUseful(mySide, state)){
                return request::DeltaRequest{types::DeltaType::SNITCH_SNATCH, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                             std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
            }else{
                return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                             std::nullopt, activeEntityId, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
            }
        }else if(activeEntityId == types::EntityId::LEFT_ELF ||
                 activeEntityId == types::EntityId::RIGHT_ELF){
            passiveEntityId = getElfTarget(mySide, state);
            if(passiveEntityId.has_value()){
                return request::DeltaRequest{types::DeltaType::ELF_TELEPORTATION, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                             std::nullopt, std::nullopt, passiveEntityId, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
            }else{
                return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                             std::nullopt, activeEntityId, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
            }
        }else if(activeEntityId == types::EntityId::LEFT_TROLL ||
                 activeEntityId == types::EntityId::RIGHT_TROLL){
            if(isTrollUseful(mySide, state)){
                return request::DeltaRequest{types::DeltaType::TROLL_ROAR, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                             std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
            }else{
                return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                             std::nullopt, activeEntityId, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
            }
        }else if(activeEntityId == types::EntityId::LEFT_GOBLIN ||
                 activeEntityId == types::EntityId::RIGHT_GOBLIN){
            passiveEntityId = getGoblinTarget(mySide, state);
            if(passiveEntityId.has_value()){
                return request::DeltaRequest{types::DeltaType::GOBLIN_SHOCK, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                             std::nullopt, std::nullopt, passiveEntityId, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
            }else{
                return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                             std::nullopt, activeEntityId, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
            }
        }else{
            return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                         std::nullopt, activeEntityId, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        }
    }

    bool isNifflerUseful(const gameModel::TeamSide &mySide, const State &state){
        if (mySide == state.env->team1->side) {
            return gameController::getDistance(state.env->team2->seeker->position, state.env->snitch->position) <= distanceSnitchSeeker;
        }else{
            return gameController::getDistance(state.env->team1->seeker->position, state.env->snitch->position) <= distanceSnitchSeeker;
        }
    }

    bool isTrollUseful(const gameModel::TeamSide &mySide, const State &state) {
        auto player = state.env->getPlayer(state.env->quaffle->position);
        if(player.has_value()) {
            if(mySide != state.env->getTeam(player.value())->side) {
                return true;
            }
        }
        return false;
    }

    auto getGoblinTarget(const gameModel::TeamSide &mySide, const State &state)->
    const std::optional<communication::messages::types::EntityId> {
        auto opponentGoals = state.env->getGoalsRight();
        if(mySide == gameModel::TeamSide::RIGHT) {
            opponentGoals = state.env->getGoalsLeft();
        }

        for(const auto &goal : opponentGoals){
            auto player = state.env->getPlayer(goal);
            if(player.has_value() && !state.env->getTeam(mySide)->hasMember(player.value())){
                return player.value()->id;
            }
        }
        return std::nullopt;
    }

    auto getElfTarget(const gameModel::TeamSide &mySide, const State &state) -> const std::optional<communication::messages::types::EntityId> {
        if(state.env->snitch->exists) {
            std::shared_ptr<gameModel::Environment> environment = state.env->clone();
            gameController::moveSnitch(environment->snitch, environment, state.overtimeState);
            if (mySide == environment->team1->side) {
                if (gameController::getDistance(environment->team2->seeker->position, environment->snitch->position) <= distanceSnitchSeeker) {
                    return environment->team2->seeker->id;
                }
            } else {
                if (gameController::getDistance(environment->team1->seeker->position, environment->snitch->position) <= distanceSnitchSeeker) {
                    return environment->team1->seeker->id;
                }
            }
        }

        return std::nullopt;
    }
}