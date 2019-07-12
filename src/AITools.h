
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
#include <SopraMessages/json.hpp>
#include <iostream>
#include <atomic>

namespace aiTools{
    constexpr auto MIN_SHOT_SUCCESS_PROB = 0.2;
    static constexpr int FIELD_WIDTH = 17;

    /**
     * Represents a complete state in the game including positions of objects and other useful data
     */
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
        std::unordered_set<communication::messages::types::EntityId> playersUsedRight = {};
        std::array<unsigned int, 5> availableFansLeft = {}; //Teleport, RangedAttack, Impulse, SnitchPush, BlockCell
        std::array<unsigned int, 5> availableFansRight = {};

        /**
         * Computes a feature vecor from its members
         * @param side side of the actin AI
         * @return array of doubles
         */
        auto getFeatureVec(gameModel::TeamSide side) const -> std::array<double, FEATURE_VEC_LEN>;

        /**
         * Returns a deep copy of itself
         * @return
         */
        State clone() const;
    };

    void to_json(nlohmann::json &j, const State &state);
    void from_json(const nlohmann::json &j, State &state);

    /**
     * Represents a generic node in a search problem
     */
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

    /**
     * Represents the type of action an entity has to take
     */
    class ActionState{
    public:
        enum class TurnState{
            FirstMove,
            SecondMove,
            Action
        };

        ActionState(communication::messages::types::EntityId id, TurnState turnState);
        communication::messages::types::EntityId id;
        TurnState turnState;
    };

    /**
     * Computes all possible ActionStates immediatly follwoing a given ActionState
     * @param state the current game state
     * @param actionState the last ActionState
     * @return List of possible ActionStates following the given ActionState along with the corresponding game state
     */
    auto computeNextActionStates(const State &state, const ActionState &actionState) -> std::vector<std::pair<State, ActionState>>;

    /**
     * Computes the best Action to respond to the given ActionState using alpha-beta-search.
     * @tparam EvalFun Type of evaluation function used for evaluating game states. Must return comparable
     * @param state The current game state
     * @param actionState the current ActionState
     * @param mySide the TeamSide the ai plays on
     * @param alpha alpha value (initially -infinity)
     * @param beta beta value (initially infinity)
     * @param maxDepth maximum search depth
     * @param evalFun evaluation function used to evaluate a State
     * @param abort atomic bool to stop computation if set to true
     * @param expansions current number of expansions during the search (initially 0)
     * @return Pair of the best possible Action and its corresponding utility value or nothing if errors occured
     */
    template <typename EvalFun>
    auto alphaBetaSearch(const State &state, const ActionState &actionState, gameModel::TeamSide mySide, double alpha,
                         double beta, int maxDepth, const EvalFun &evalFun, const std::atomic_bool &abort, unsigned long &expansions) ->
                         std::optional<std::pair<std::shared_ptr<gameController::Action>, double>> {
        expansions++;
        std::vector<std::shared_ptr<gameController::Action>> allActions;
        auto currentPlayer = state.env->getPlayerById(actionState.id);
        if(actionState.turnState == ActionState::TurnState::Action){
            auto actionType = gameController::getPossibleBallActionType(currentPlayer, state.env);
            if(!actionType.has_value()){
                std::cerr << "-------No action possible-------" << std::endl;
                return std::nullopt;
            }

            if(*actionType == gameController::ActionType::Throw){
                auto tmp = gameController::getAllConstrainedShots(currentPlayer, state.env);
                for(const auto &a : tmp){
                    allActions.emplace_back(std::make_shared<gameController::Shot>(a));
                }
            } else {
                auto chaser = std::dynamic_pointer_cast<gameModel::Chaser>(currentPlayer);
                if(!chaser){
                    throw std::runtime_error("Player is no chaser");
                }

                allActions.emplace_back(std::make_shared<gameController::WrestQuaffle>(gameController::WrestQuaffle(state.env, chaser, state.env->quaffle->position)));
            }
        } else {
            auto tmp = gameController::getAllPossibleMoves(currentPlayer, state.env);
            allActions.reserve(tmp.size());
            for(const auto &a : tmp){
                allActions.emplace_back(std::make_shared<gameController::Move>(a));
            }
        }


        double minMaxVal = std::numeric_limits<double>::infinity();
        std::optional<std::shared_ptr<gameController::Action>> minMaxAction;
        bool maxSearch = gameLogic::conversions::idToSide(actionState.id) == mySide;
        if(maxSearch){
            minMaxVal *= -1;
        }

        if(allActions.empty()){
            std::cerr << "---------Action list is empty. Depth: " + std::to_string(maxDepth) << std::endl;
            return std::nullopt;
        }

        for(const auto &action : allActions){
            double expectedValue = 0;
            for(const auto &outcome : action->executeAll()){
                const auto &currentEnv = outcome.first;
                auto newState = state.clone();
                newState.env = outcome.first;
                newState.goalScoredThisRound = state.env->team1->score != newState.env->team1->score || state.env->team2->score != newState.env->team2->score;
                auto playerOnSnitch = currentEnv->getPlayer(currentEnv->snitch->position);
                if(abort || maxDepth == 0 || (playerOnSnitch.has_value() && INSTANCE_OF(*playerOnSnitch, gameModel::Seeker))){
                    return std::make_pair(action, evalFun(newState));
                }

                double currentOutcomeExpectedValue = 0;
                auto nextActors = computeNextActionStates(newState, actionState);
                for(const auto &nextActor : nextActors){
                    auto tmp = alphaBetaSearch(nextActor.first, nextActor.second, mySide, alpha, beta, maxDepth - 1, evalFun, abort, expansions);
                    if(!tmp.has_value()){
                        return std::make_pair(action, evalFun(newState));
                    }

                    currentOutcomeExpectedValue += tmp->second;
                    //Bedingung kann hier zutreffen, da abort von außen verändert werden kann
                    if(abort){
                        return std::make_pair(action, evalFun(newState));
                    }
                }

                currentOutcomeExpectedValue /= nextActors.size();
                expectedValue += currentOutcomeExpectedValue * outcome.second;
            }

            if(maxSearch){
                if(expectedValue > minMaxVal){
                    minMaxVal = expectedValue;
                    minMaxAction.emplace(action);
                }

                if(expectedValue >= beta){
                    if(!minMaxAction.has_value()){
                        std::cerr << "----------error during pruning in max, no value. Depth: " + std::to_string(maxDepth) << std::endl;
                        return std::nullopt;
                    }

                    return std::make_pair(minMaxAction.value(), minMaxVal);
                }

                alpha = std::max(alpha, expectedValue);

            } else {
                if(expectedValue < minMaxVal){
                    minMaxVal = expectedValue;
                    minMaxAction.emplace(action);
                }

                if(expectedValue <= alpha){
                    if(!minMaxAction.has_value()){
                        std::cerr << "----------error during pruning in min, no value. Depth: " + std::to_string(maxDepth) << std::endl;
                        return std::nullopt;
                    }

                    return std::make_pair(minMaxAction.value(), minMaxVal);
                }

                beta = std::min(beta, expectedValue);
            }
        }

        if(!minMaxAction.has_value()){
            std::cerr << "----------no value after all actions. Depth: " + std::to_string(maxDepth) << std::endl;
            return std::nullopt;
        }

        return std::make_pair(minMaxAction.value(), minMaxVal);
    }

    /**
     * Computes a DeltaRequest containing the best action possible in a current state using alpha beta search as iterative deepening search.
     * @tparam EvalFun Type of evaluation function used for evaluating game states. Must return comparable
     * @param state current game state
     * @param evalFun evaluation function used to evaluate a State
     * @param actionState current action state defining id and type of action to take
     * @param abort atomic bool flag to abort computation
     * @param minDepth minimal search depth
     * @return DeltaRequest containing the desired action, the search depth on which the result was aquired, total number of states explored
     */
    template <typename EvalFun>
    auto computeBestActionAlphaBetaID(const State &state, const EvalFun &evalFun, const ActionState &actionState, const std::atomic_bool &abort, int minDepth) ->
        std::tuple<communication::messages::request::DeltaRequest, int, unsigned long> {
        using namespace communication::messages;
        int maxDepth = minDepth;
        std::optional<request::DeltaRequest> bestAction;
        double bestScore = 0;
        unsigned long totalExpansions = 0;
        while (!abort){
            auto [action, score, expansions] = computeBestActionAlphaBeta(state, evalFun, actionState, maxDepth++, abort);
            if(!bestAction.has_value() || score > bestScore){
                bestAction.emplace(action);
                bestScore = score;
                totalExpansions += expansions;
            }
        }

        if(bestAction.has_value()){
            return {*bestAction, maxDepth, totalExpansions};
        }

        return {request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, actionState.id,
                                     std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt}, maxDepth, totalExpansions};
    }

    /**
     * Computes a DeltaRequest containing the best action possible in a current state using alpha beta search.
     * @tparam EvalFun Type of evaluation function used for evaluating game states. Must return comparable
     * @param state current game state
     * @param evalFun evaluation function used to evaluate a State
     * @param actionState current action state defining id and type of action to take
     * @param maxDepth maximum search depth
     * @param abort atomic bool flag to abort computation
     * @return pair containing DeltaRequest and corresponding utility value
     */
    template <typename EvalFun>
    auto computeBestActionAlphaBeta(const State &state, const EvalFun &evalFun, const ActionState &actionState, int maxDepth, const std::atomic_bool &abort) ->
        std::tuple<communication::messages::request::DeltaRequest, double, unsigned long> {
        double currentScore = evalFun(state);
        using namespace communication::messages;
        unsigned long expansions = 0;
        auto res = alphaBetaSearch(state, actionState, gameLogic::conversions::idToSide(actionState.id),
                -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), maxDepth, evalFun, abort, expansions);
        if(!res.has_value()){
            return {request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, actionState.id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt}, currentScore, expansions};
        }

        auto [bestAction, bestScore] = *res;

        if(bestScore > currentScore){
            if(INSTANCE_OF(bestAction, gameController::Move)){
                return {request::DeltaRequest{types::DeltaType::MOVE, std::nullopt, std::nullopt, std::nullopt, bestAction->getTarget().x,
                                             bestAction->getTarget().y, actionState.id, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt}, bestScore, expansions};
            } else if(INSTANCE_OF(bestAction, gameController::Shot)){
                auto shot = std::dynamic_pointer_cast<gameController::Shot>(bestAction);
                if(INSTANCE_OF(shot->getBall(), const gameModel::Quaffle)){
                    return {request::DeltaRequest{types::DeltaType::QUAFFLE_THROW, std::nullopt, std::nullopt, std::nullopt, shot->getTarget().x,
                                                 shot->getTarget().y, actionState.id, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt}, bestScore, expansions};
                } else if(INSTANCE_OF(shot->getBall(), const gameModel::Bludger)){
                    return {request::DeltaRequest{types::DeltaType::BLUDGER_BEATING, std::nullopt, std::nullopt, std::nullopt, shot->getTarget().x,
                                                 shot->getTarget().y, actionState.id, shot->getBall()->getId(), std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt}, bestScore, expansions};
                } else {
                    throw std::runtime_error("Invalid shot was calculated");
                }
            } else {
                return {request::DeltaRequest{types::DeltaType::WREST_QUAFFLE, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt,
                                             actionState.id, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt}, bestScore, expansions};
            }
        } else {
            return {request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, actionState.id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt}, currentScore, expansions};
        }
    }



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
     * @param abort flag used to abort the computation. If set to true, the currently best result will be returned
     * @return tuple of the best Action and its score or nothing if no result was found in time
     */
    template <typename ActionType, typename EvalFun>
    auto chooseBestAction(const std::vector<ActionType> &actionList, const EvalFun &evalFun, const bool &abort) ->
        std::optional<std::tuple<typename std::vector<ActionType>::const_iterator, double>>{
        std::optional<std::tuple<typename std::vector<ActionType>::const_iterator, double>> ret;
        if(actionList.empty()){
            throw std::runtime_error("List is empty. Cannot choose best entry");
        }

        double highestScore = -std::numeric_limits<double>::infinity();
        std::optional<typename std::vector<ActionType>::const_iterator> best;
        for(auto action = actionList.begin(); action < actionList.end(); ++action){
            if(abort){
                break;
            }

            double tmpScore = 0;
            for(const auto &outcome : action->executeAll()){
                tmpScore += outcome.second * evalFun(outcome.first);
            }

            if(tmpScore > highestScore){
                highestScore = tmpScore;
                best.emplace(action);
            }
        }

        if(best.has_value()){
            ret.emplace(*best, highestScore);
        }

        return ret;
    }

    /**
     * Computes the next move according to the current state of the game
     * @tparam EvalFun type of evaluation function for a state. Must return a comparable type
     * @param state the current game state
     * @param id the ID of the player to make a move
     * @param abort flag used to abort the computation.
     * @throws std::runtime_error when no move is possible
     * @return next move as DeltaRequest
     */
    template <typename EvalFun>
    auto computeBestMove(const State &state, const EvalFun &evalFun, communication::messages::types::EntityId id, const bool &abort) ->
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

        auto res = aiTools::chooseBestAction(moves, envEvalFun, abort);
        if(!res.has_value()){
            return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        }

        auto [best, score] = *res;
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
     * @param abort flag used to abort the computation.
     * @throws std::runtime_error when no shot is possible
     * @return next shot as DeltaRequest
     */
    template <typename EvalFun>
    auto computeBestShot(const State &state, const EvalFun &evalFun, communication::messages::types::EntityId id, const bool &abort) ->
        communication::messages::request::DeltaRequest{
        using namespace communication::messages;
        auto player = state.env->getPlayerById(id);
        auto shots = gameController::getAllPossibleShots(player, state.env, MIN_SHOT_SUCCESS_PROB);
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

        auto res = aiTools::chooseBestAction(shots, envEvalFun, abort);
        if(!res.has_value()){
            return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        }

        auto [best, score] = *res;
        if(score < evalFun(state)){
            return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        }

        if(INSTANCE_OF(best->getBall(), const gameModel::Quaffle)){
            return request::DeltaRequest{types::DeltaType::QUAFFLE_THROW, std::nullopt, std::nullopt, std::nullopt, best->getTarget().x,
                                         best->getTarget().y, id, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
        } else if(INSTANCE_OF(best->getBall(), const gameModel::Bludger)){
            return request::DeltaRequest{types::DeltaType::BLUDGER_BEATING, std::nullopt, std::nullopt, std::nullopt, best->getTarget().x,
                                         best->getTarget().y, id, best->getBall()->getId(), std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
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
    auto computeBestWrest(const State &state, const EvalFun &evalFun, communication::messages::types::EntityId id) ->
        communication::messages::request::DeltaRequest{
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
     * @param abort flag used to abort the computation.
     * @return unban request as DeltaRequest
     */
    template <typename EvalFun>
    auto redeployPlayer(const State &state, const EvalFun &evalFun, communication::messages::types::EntityId id, const bool &abort)
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
            if(abort){
                break;
            }

            auto newEnv = state.env->clone();
            newEnv->getPlayerById(id)->position = pos;
            newEnv->getPlayerById(id)->isFined = false;
            auto score = envEvalFun(newEnv);
            if(score > bestScore) {
                bestScore = score;
                redeployPos = pos;
            }
        }

        if(redeployPos == gameModel::Position{0, 0}){
            return request::DeltaRequest{types::DeltaType::SKIP, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, id,
                                         std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
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
