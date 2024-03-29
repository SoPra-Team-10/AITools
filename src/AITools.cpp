//
// Created by tim on 14.06.19.
//

#include "AITools.h"
#include <SopraGameLogic/SharedPtrSerialization.h>

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
            for (const auto &pos : env->getAllLegalCellsAround(node.state, env->getTeam(gameModel::TeamSide::LEFT)->hasMember(player))) {
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
        if (mySide == state.env->team1->getSide()) {
            return gameController::getDistance(state.env->team2->seeker->position, state.env->snitch->position) <= distanceSnitchSeeker;
        }else{
            return gameController::getDistance(state.env->team1->seeker->position, state.env->snitch->position) <= distanceSnitchSeeker;
        }
    }

    bool isTrollUseful(const gameModel::TeamSide &mySide, const State &state) {
        auto player = state.env->getPlayer(state.env->quaffle->position);
        if(player.has_value()) {
            if(mySide != state.env->getTeam(player.value())->getSide()) {
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
                return player.value()->getId();
            }
        }
        return std::nullopt;
    }

    auto getElfTarget(const gameModel::TeamSide &mySide, const State &state) -> const std::optional<communication::messages::types::EntityId> {
        if(state.env->snitch->exists) {
            std::shared_ptr<gameModel::Environment> environment = state.env->clone();
            gameController::moveSnitch(environment->snitch, environment, state.overtimeState);
            if (mySide == environment->team1->getSide()) {
                if (gameController::getDistance(environment->team2->seeker->position, environment->snitch->position) <= distanceSnitchSeeker) {
                    return environment->team2->seeker->getId();
                }
            } else {
                if (gameController::getDistance(environment->team1->seeker->position, environment->snitch->position) <= distanceSnitchSeeker) {
                    return environment->team1->seeker->getId();
                }
            }
        }

        return std::nullopt;
    }

    auto getTeamFormation(gameModel::TeamSide side) -> communication::messages::request::TeamFormation {
        using P = gameModel::Position;
        P seekerPos{3, 8};
        P keeperPos{3, 6};
        P c1Pos{7, 4};
        P c2Pos{6, 6};
        P c3Pos{7, 8};
        P b1Pos{6, 5};
        P b2Pos{6, 7};
        if(side == gameModel::TeamSide::RIGHT){
            mirrorPos(seekerPos);
            mirrorPos(keeperPos);
            mirrorPos(c1Pos);
            mirrorPos(c2Pos);
            mirrorPos(c3Pos);
            mirrorPos(b1Pos);
            mirrorPos(b2Pos);
        }

        return {seekerPos.x, seekerPos.y, keeperPos.x, keeperPos.y, c1Pos.x, c1Pos.y, c2Pos.x, c2Pos.y,
                c3Pos.x, c3Pos.y, b1Pos.x, b1Pos.y, b2Pos.x, b2Pos.y};
    }

    void mirrorPos(gameModel::Position &pos) {
        pos.x = FIELD_WIDTH - 1 - pos.x;
    }

    int maybeMirrorX(const int &x, bool necessary) {
        if(necessary){
            return FIELD_WIDTH - 1 - x;
        } else {
            return x;
        }
    }

    void to_json(nlohmann::json &j, const State &state) {
        j["environment"] = state.env;
        j["round"] = state.roundNumber;
        j["phase"] = state.currentPhase;
        j["overTimeState"] = state.overtimeState;
        j["overTimeCounter"] = state.overTimeCounter;
        j["goalScoredThisRound"] = state.goalScoredThisRound;
        j["playersUsedLeft"] = state.playersUsedLeft;
        j["playersUsedRight"] = state.playersUsedRight;
        j["fansLeft"] = state.availableFansLeft;
        j["fansRight"] = state.availableFansRight;
    }

    void from_json(const nlohmann::json &j, State &state) {
        using namespace communication::messages::types;
        state.env = j.at("environment").get<std::shared_ptr<gameModel::Environment>>();
        state.roundNumber = j.at("round").get<unsigned int>();
        state.currentPhase = j.at("phase").get<PhaseType>();
        state.overtimeState = j.at("overTimeState").get<gameController::ExcessLength>();
        state.overTimeCounter = j.at("overTimeCounter").get<unsigned int>();
        state.playersUsedLeft = j.at("playersUsedLeft").get<std::unordered_set<EntityId>>();
        state.playersUsedRight = j.at("playersUsedRight").get<std::unordered_set<EntityId>>();
        state.availableFansLeft = j.at("fansLeft").get<std::array<unsigned int, 5>>();
        state.availableFansRight = j.at("fansRight").get<std::array<unsigned int, 5>>();
    }

    auto computeNextActionStates(const State &state,
                                 const ActionState &actionState) -> std::vector<std::pair<State, ActionState>> {
        using namespace gameLogic::conversions;
        auto currentPlayer = state.env->getPlayerById(actionState.id);
        std::vector<std::pair<State, ActionState>> ret;
        bool secondMoveProbable = !currentPlayer->isFined && !currentPlayer->knockedOut && actionState.turnState == ActionState::TurnState::FirstMove &&
            state.env->config.getExtraTurnProb(state.env->getPlayerById(actionState.id)->broom) > 0.5;
        bool newPlayerNeeded = currentPlayer->isFined || currentPlayer->knockedOut || actionState.turnState == ActionState::TurnState::Action ||
            !gameController::playerCanPerformAction(state.env->getPlayerById(actionState.id), state.env);
        if(secondMoveProbable){
            ret.emplace_back(state.clone(), ActionState{actionState.id, ActionState::TurnState::SecondMove});
        } else if(newPlayerNeeded){
            std::vector<communication::messages::types::EntityId> nextPlayers;
            auto &usedPlayers = idToSide(actionState.id) == gameModel::TeamSide::LEFT ? state.playersUsedRight : state.playersUsedLeft;
            ret.reserve(7 - usedPlayers.size());
            auto nextSide = idToSide(actionState.id) == gameModel::TeamSide::LEFT ? gameModel::TeamSide::RIGHT : gameModel::TeamSide::LEFT;
            for(const auto &player : state.env->getTeam(nextSide)->getAllPlayers()){
                if(player->isFined || player->knockedOut){
                    continue;
                }

                bool used = false;
                for(auto id : usedPlayers){
                    if(player->getId() == id){
                        used = true;
                        break;
                    }
                }

                if(used){
                    continue;
                }

                //Copy curent state
                auto newState = state.clone();
                //Insert last player as used
                auto &usedPlayersNew = idToSide(actionState.id) == gameModel::TeamSide::LEFT ? newState.playersUsedLeft : newState.playersUsedRight;
                usedPlayersNew.emplace(actionState.id);
                ret.emplace_back(newState, ActionState{player->getId(), ActionState::TurnState::FirstMove});
            }
        } else {
            ret.emplace_back(state.clone(), ActionState{actionState.id, ActionState::TurnState::Action});
        }

        return ret;
    }


    auto State::getFeatureVec(gameModel::TeamSide side) const -> std::array<double, 122> {
        std::array<double, FEATURE_VEC_LEN> ret = {};
        bool mirror = side == gameModel::TeamSide::RIGHT;
        auto insertTeam = [this, mirror](gameModel::TeamSide side, std::array<double, 120>::iterator &it){
            auto &usedPlayers = side == gameModel::TeamSide::LEFT ? playersUsedLeft : playersUsedRight;
            auto &availableFans = side == gameModel::TeamSide::LEFT ? availableFansLeft : availableFansRight;
            for(const auto &player : env->getTeam(side)->getAllPlayers()){
                *it++ = maybeMirrorX(player->position.x, mirror);
                *it++ = player->position.y;
                bool used = false;
                for(const auto &id : usedPlayers){
                    if(player->getId() == id){
                        used = true;
                        break;
                    }
                }

                *it++ = used;
                *it++ = !used && !player->knockedOut && !player->isFined;
                *it++ = player->knockedOut;
                *it++ = player->isFined;
            }

            for(const auto &useNumber : availableFans){
                *it++ = useNumber;
            }
        };

        auto opponentSide = side == gameModel::TeamSide::LEFT ? gameModel::TeamSide::RIGHT : gameModel::TeamSide::LEFT;
        ret[0] = roundNumber;
        ret[1] = static_cast<double>(currentPhase);
        ret[2] = static_cast<double>(overtimeState);
        ret[3] = overTimeCounter;
        ret[4] = goalScoredThisRound;
        ret[5] = env->getTeam(side)->score;
        ret[6] = env->getTeam(opponentSide)->score;
        auto it = ret.begin() + 7;
        for(const auto &shit : env->pileOfShit){
            *it++ = maybeMirrorX(shit->position.x, mirror);
            *it++ = shit->position.y;
        }

        for(unsigned long i = 0; i < 12 - env->pileOfShit.size(); i++){
            *it++ = 0;
            *it++ = 0;
        }

        ret[19] = maybeMirrorX(env->quaffle->position.x, mirror);
        ret[20] = env->quaffle->position.y;
        ret[21] = maybeMirrorX(env->bludgers[0]->position.x, mirror);
        ret[22] = env->bludgers[0]->position.y;
        ret[23] = maybeMirrorX(env->bludgers[1]->position.x, mirror);
        ret[24] = env->bludgers[1]->position.y;
        ret[25] = maybeMirrorX(env->snitch->position.x, mirror);
        ret[26] = env->snitch->position.y;
        ret[27] = env->snitch->exists;
        it = ret.begin() + 28;
        insertTeam(side, it);
        insertTeam(opponentSide, it);
        return ret;
    }

    State State::clone() const {
        State ret = *this;
        ret.env = this->env->clone();
        return ret;
    }

    ActionState::ActionState(communication::messages::types::EntityId id, ActionState::TurnState turnState) :
        id(id), turnState(turnState) {}
}