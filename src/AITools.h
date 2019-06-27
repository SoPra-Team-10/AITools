
//
// Created by tim on 14.06.19.
//

#ifndef KI_AITOOLS_H
#define KI_AITOOLS_H

#include <memory>
#include <optional>
#include <deque>
#include <list>
#include <functional>
#include <set>
#include <SopraGameLogic/Action.h>
#include <SopraGameLogic/conversions.h>
#include <SopraMessages/Next.hpp>
#include <SopraMessages/DeltaRequest.hpp>
#include <unordered_set>

namespace aiTools{
    constexpr auto minShotSuccessProb = 0.2;
    static constexpr int FIELD_WIDTH = 16;

    class State {
    public:
        static constexpr auto FEATURE_VEC_LEN = 122;
        std::shared_ptr<gameModel::Environment> env;
        unsigned int roundNumber = 1;
        communication::messages::types::PhaseType currentPhase = communication::messages::types::PhaseType::BALL_PHASE;
        gameController::ExcessLength overtimeState = gameController::ExcessLength::None;
        unsigned int overTimeCounter = 0;
        bool goalScoredThisRound = false;
        std::unordered_set<communication::messages::types::EntityId> playersUsedLeft = {};
        std::unordered_set<communication::messages::types::EntityId> playersUsedRight ={};
        std::array<unsigned int, 5> availableFansLeft = {}; //Teleport, RangedAttack, Impulse, SnitchPush, BlockCell
        std::array<unsigned int, 5> availableFansRight = {};

        /**
         * Computes a feature vecor from its members
         * @param side side of the actin AI
         * @return array of doubles
         */
        auto getFeatureVec(gameModel::TeamSide side) const -> std::array<double, FEATURE_VEC_LEN>;
    };

    template <typename T>
    class SearchNode {
    public:
        SearchNode(T state, std::optional<std::shared_ptr<SearchNode<T>>> parent, int pathCost) :
            state(std::move(state)), parent(std::move(parent)), pathCost(pathCost) {}

        const T state;
        std::optional<std::shared_ptr<SearchNode<T>>> parent;
        int pathCost = 0;
        bool operator==(const SearchNode<T> &other) const;
    };


    template<typename T>
    bool SearchNode<T>::operator==(const SearchNode<T> &other) const {
        return this->state == other.state;
    }


    /**
     * A*algorithm implemented as graph search
     * @tparam T Type of the SearchNode used
     * @tparam EXP Type of expand function used, must return iterable of SearchNodes<T>
     * @tparam F Type of evaluation function for nodes. Must return comparable value
     * @param Start initial state
     * @param Destination goal state
     * @param expand Function used for expanding nodes
     * @param f Function used for evaluating a node. A high value corresponds to low utility
     * @return The goal node with information about its parent or nothing if problem is impossible
     */
    template <typename T, typename Exp, typename F>
    auto aStarSearch(const SearchNode<T> &start, const SearchNode<T> &destination,
            const Exp &expand, const F &f) -> std::optional<SearchNode<T>>{
        std::deque<SearchNode<T>> visited;
        std::list<SearchNode<T>> fringe = {start};
        std::optional<SearchNode<T>> ret;
        while(!fringe.empty()){
            const auto currentNode = *fringe.begin();
            fringe.pop_front();
            if(currentNode == destination){
                ret.emplace(currentNode);
                break;
            }

            bool nodeVisited = false;
            for(const auto &node : visited){
                if(currentNode == node){
                    nodeVisited = true;
                    break;
                }
            }

            if(nodeVisited){
                continue;
            }

            visited.emplace_back(currentNode);
            for(const auto &newNode : expand(currentNode)){
                bool inserted = false;
                for(auto it = fringe.begin(); it != fringe.end(); ++it){
                    if(f(newNode) < f(*it)){
                        fringe.emplace(it, newNode);
                        inserted = true;
                        break;
                    }
                }

                if(!inserted){
                    fringe.emplace_back(newNode);
                    fringe.size();
                }
            }
        }

        return ret;
    }

    /**
     * Selects the best Action from a list of Actions.
     * @tparam ActionType type of Action
     * @tparam EvalFun type of function used for evaluating the actions' outcomes
     * @param actionList list of actions to evaluate
     * @param evalFun evaluation function for comparing outcomes of actions
     * @return tuple of the best Action and its score
     */
    template <typename ActionType, typename EvalFun>
    auto chooseBestAction(const std::vector<ActionType> &actionList, const EvalFun &evalFun) ->
        std::tuple<typename std::vector<ActionType>::const_iterator, double>{
        if(actionList.empty()){
            throw std::runtime_error("List is empty. Cannot choose best entry");
        }

        double highestScore = -std::numeric_limits<double>::infinity();
        std::optional<typename std::vector<ActionType>::const_iterator> best;
        for(auto action = actionList.begin(); action < actionList.end(); ++action){
            double tmpScore = 0;
            for(const auto &outcome : action->executeAll()){
                tmpScore += outcome.second * evalFun(outcome.first);
            }

            if(tmpScore > highestScore){
                highestScore = tmpScore;
                best.emplace(action);
            }
        }

        if(!best.has_value()){
            throw std::runtime_error("No best candidate found");
        }

        return {best.value(), highestScore};
    }

    /**
     * Computes the next move according to the current state of the game
     * @tparam EvalFun type of evaluation function for a state. Must return a comparable type
     * @param state the current game state
     * @param id the ID of the player to make a move
     * @throws std::runtime_error when no move is possible
     * @return next move as DeltaRequest
     */
    template <typename EvalFun>
    auto computeBestMove(const State &state, const EvalFun &evalFun, communication::messages::types::EntityId id) ->
        communication::messages::request::DeltaRequest{
        using namespace communication::messages;
        auto player = state.env->getPlayerById(id);
        auto moves = gameController::getAllPossibleMoves(player, state.env);
        if(moves.empty()){
            throw std::runtime_error("No move possible");
        }

        auto envEvalFun = [&state, &evalFun](const std::shared_ptr<gameModel::Environment> &env){
            auto stateTmp = state;
            stateTmp.env = env;
            return evalFun(stateTmp);
        };

        auto [best, score] = aiTools::chooseBestAction(moves, envEvalFun);
        if(score < evalFun(state)) {
            return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        }

        return request::DeltaRequest{types::DeltaType::MOVE, std::nullopt, std::nullopt, std::nullopt, best->getTarget().x,
                                     best->getTarget().y, id, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
    }

    /**
     * Computes the next shot according to the current state of the game
     * @tparam EvalFun type of evaluation function for a state. Must return a comparable type
     * @param state the current game state
     * @param evalFun the function used for evaluation a game state
     * @param id the ID of the player to perform a shot
     * @throws std::runtime_error when no shot is possible
     * @return next shot as DeltaRequest
     */
    template <typename EvalFun>
    auto computeBestShot(const State &state, const EvalFun &evalFun, communication::messages::types::EntityId id) ->
        communication::messages::request::DeltaRequest{
        using namespace communication::messages;
        auto player = state.env->getPlayerById(id);
        auto shots = gameController::getAllPossibleShots(player, state.env, minShotSuccessProb);
        if(shots.empty()){
            //No shot with decent success probability possible -> skip turn
            return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        }

        auto envEvalFun = [&state, &evalFun](const std::shared_ptr<gameModel::Environment> &env){
            auto stateTmp = state;
            stateTmp.env = env;
            return evalFun(stateTmp);
        };

        auto [best, score] = aiTools::chooseBestAction(shots, envEvalFun);
        if(score < evalFun(state)){
            return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        }

        if(INSTANCE_OF(best->getBall(), const gameModel::Quaffle)){
            return request::DeltaRequest{types::DeltaType::QUAFFLE_THROW, std::nullopt, std::nullopt, std::nullopt, best->getTarget().x,
                                         best->getTarget().y, id, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        } else if(INSTANCE_OF(best->getBall(), const gameModel::Bludger)){
            return request::DeltaRequest{types::DeltaType::BLUDGER_BEATING, std::nullopt, std::nullopt, std::nullopt, best->getTarget().x,
                                         best->getTarget().y, id, best->getBall()->id, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        } else {
            throw std::runtime_error("Invalid shot was calculated");
        }
    }


    /**
     * Computes the next wrest according to the current state of the game
     * @tparam EvalFun type of evaluation function for a state. Must return a comparable type
     * @param state the current game state
     * @param evalFun the function used for evaluation a game state
     * @param id the ID of the player to perform a shot
     * @throws std::runtime_error when no shot is possible
     * @return next wrest as DeltaRequest
     */
    template <typename EvalFun>
    auto computeBestWrest(const State &state, const EvalFun &evalFun, communication::messages::types::EntityId id) -> communication::messages::request::DeltaRequest{
        using namespace communication::messages;
        auto player = std::dynamic_pointer_cast<gameModel::Chaser>(state.env->getPlayerById(id));
        if(!player){
            throw std::runtime_error("Player is no Chaser");
        }

        gameController::WrestQuaffle wrest(state.env, player, state.env->quaffle->position);
        if(wrest.check() == gameController::ActionCheckResult::Impossible){
            throw std::runtime_error("Wrest is impossible");
        }

        auto envEvalFun = [&state, &evalFun](const std::shared_ptr<gameModel::Environment> &env){
            auto stateTmp = state;
            stateTmp.env = env;
            return evalFun(stateTmp);
        };

        auto currentScore = evalFun(state);
        double wrestScore = 0;
        for(const auto &outcome : wrest.executeAll()){
            wrestScore += outcome.second * envEvalFun(outcome.first);
        }

        if(currentScore < wrestScore){
            return request::DeltaRequest{types::DeltaType::WREST_QUAFFLE, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                         id, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        } else {
            return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        }
    }

    /**
     * Computes the Position where the banned Player should be redeployed
     * @tparam EvalFun type of evaluation function for a state. Must return a comparable type
     * @param state the current game state
     * @param evalFun the function used for evaluation a game state
     * @param id the ID of the player to perform a shot
     * @return unban request as DeltaRequest
     */
    template <typename EvalFun>
    auto redeployPlayer(const State &state, const EvalFun &evalFun, communication::messages::types::EntityId id)
    -> communication::messages::request::DeltaRequest{
        auto envEvalFun = [&state, &evalFun](const std::shared_ptr<gameModel::Environment> &env){
            auto stateTmp = state;
            stateTmp.env = env;
            return evalFun(stateTmp);
        };

        using namespace communication::messages;
        auto mySide = gameLogic::conversions::idToSide(id);
        auto bestScore = -std::numeric_limits<double>::infinity();
        gameModel::Position redeployPos(0, 0);
        for(const auto &pos : state.env->getFreeCellsForRedeploy(mySide)){
            auto newEnv = state.env->clone();
            newEnv->getPlayerById(id)->position = pos;
            newEnv->getPlayerById(id)->isFined = false;
            auto score = envEvalFun(newEnv);
            if(score > bestScore) {
                bestScore = score;
                redeployPos = pos;
            }
        }


        return request::DeltaRequest{types::DeltaType::UNBAN, std::nullopt, std::nullopt, std::nullopt, redeployPos.x, redeployPos.y, id,
                                     std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
    }

    /**
     * Computes the optimal path from start to destination avoiding possible fouls using A*-search
     * @param start The Position to start
     * @param destination The desired gol Position
     * @param env The Environment to operate on
     * @return The optimal path as a list of Positions ordered from start (vector::back) to destination (vector::front).
     * Start an destination are inclusivie. Empty if start = destination or when no path exists
     */
    auto computeOptimalPath(const std::shared_ptr<const gameModel::Player> &player, const gameModel::Position &destination,
                            const std::shared_ptr<const gameModel::Environment> &env) -> std::vector<gameModel::Position>;

    /**
     * evaluates if it is necessary to use a fan
     * @param state the current game state
     * @param next gives the next EntityID
     * @return returns the new DeltaRequest
     */
    auto getNextFanTurn(const State &state, const communication::messages::broadcast::Next &next) -> const communication::messages::request::DeltaRequest;

    /**
     * evaluates, if it is useful to use a Niffler as a Fan
     * @param mySide is the Teamside from the AI
     * @param state the current game state
     * @return true, if it is useful, to use a Niffler, otherwise false
     */
    bool isNifflerUseful(const gameModel::TeamSide &mySide, const State &state);

    /**
     * evaluates, if it is useful to use an Elf as a Fan
     * @param mySide  is the Teamside from the AI
     * @param state the current game state
     * @return returns an EntityID from the Target, which should be teleported
     */
    auto getElfTarget(const gameModel::TeamSide &mySide, const State &state) -> const std::optional<communication::messages::types::EntityId>;

    /**
    * evaluates, if it is useful to use a Troll as a Fan
    * @param mySide is the Teamside from the AI
     * @param state the current game state
    * @return true, if it is useful, to use a Troll, otherwise false
    */
    bool isTrollUseful(const gameModel::TeamSide &mySide, const State &state);

    /**
    * evaluates, if it is useful to use a Goblin as a Fan
    * @param mySide is the Teamside from the AI
     * @param state the current game state
    * @return returns an EntityID from the Target, which be attacked by the Range-Attack
    */
    auto getGoblinTarget(const gameModel::TeamSide &mySide, const State &state) -> const std::optional<communication::messages::types::EntityId>;

    /**
     * Returns the AI s initial formation
     * @param side the side on which the AI plays
     * @return
     */
    auto getTeamFormation(gameModel::TeamSide side) -> communication::messages::request::TeamFormation;

    /**
     * Mirrors the given position in place on the x-axis
     * @param pos
     */
    void mirrorPos(gameModel::Position &pos);

    int maybeMirrorX(const int &x, bool necessary);
}

#endif //KI_AITOOLS_H
