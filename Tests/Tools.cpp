//
// Created by timluchterhand on 26.06.19.
//

#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>
#include "setup.h"
#include "AITools.h"
#include <SopraGameLogic/GameController.h>
#include <SopraGameLogic/GameModel.h>
#include <SopraGameLogic/Interference.h>
//-----------------------------------optimal path search----------------------------------------------------------------


TEST(ai_test, optimal_path){
    auto env = setup::createEnv();
    auto path = aiTools::computeOptimalPath(env->team2->seeker, {8, 6}, env);
    EXPECT_EQ(path.size(), 4);
    EXPECT_EQ(path.front(), gameModel::Position(8, 6));
    EXPECT_EQ(path.back(), env->team2->seeker->position);
    for(unsigned long i = 1; i < path.size(); i++){
        EXPECT_EQ(gameController::getDistance(path[i - 1], path[i]), 1);
    }
}

TEST(ai_test, optimal_path_long){
    auto env = setup::createEnv();
    auto path = aiTools::computeOptimalPath(env->team2->keeper, {0, 4}, env);
    EXPECT_EQ(path.size(), 14);
    EXPECT_EQ(path.front(), gameModel::Position(0, 4));
    EXPECT_EQ(path.back(), env->team2->keeper->position);
    for(unsigned long i = 1; i < path.size(); i++){
        EXPECT_EQ(gameController::getDistance(path[i - 1], path[i]), 1);
    }
}

TEST(ai_test, optimal_path_blocked){
    auto env = setup::createEnv();
    env->team1->seeker->position = {14, 11};
    env->team1->beaters[0]->position = {13, 11};
    auto path = aiTools::computeOptimalPath(env->team2->keeper, {15, 9}, env);
    EXPECT_EQ(path.size(), 7);
    EXPECT_EQ(path.front(), gameModel::Position(15, 9));
    EXPECT_EQ(path.back(), env->team2->keeper->position);
    for(unsigned long i = 1; i < path.size(); i++){
        EXPECT_EQ(gameController::getDistance(path[i - 1], path[i]), 1);
    }
}

TEST(ai_test, optimal_path_blocked_complex){
    auto env = setup::createEnv();
    env->team1->seeker->position = {9, 5};
    env->team1->chasers[0]->position = {7, 5};
    env->team1->chasers[2]->position = {9, 6};
    env->team1->beaters[0]->position = {7, 6};
    env->team2->keeper->position = {8, 7};
    auto path = aiTools::computeOptimalPath(env->team2->keeper, {9, 4}, env);
    EXPECT_EQ(path.size(), 5);
    EXPECT_EQ(path.front(), gameModel::Position(9, 4));
    EXPECT_EQ(path.back(), env->team2->keeper->position);
    for(unsigned long i = 1; i < path.size(); i++){
        EXPECT_EQ(gameController::getDistance(path[i - 1], path[i]), 1);
    }
}

TEST(ai_test, optimal_path_impossible){
    auto env = setup::createEnv();
    auto path = aiTools::computeOptimalPath(env->team2->seeker, env->team1->keeper->position, env);
    EXPECT_EQ(path.size(), 0);
}

//------------------------------------------Interference Useful Tests--------------------------------------------------

TEST(ai_test, getNextFanTurn0){
    using namespace communication::messages;
    auto env = setup::createEnv();
    aiTools::State state;
    state.env = env;
    broadcast::Next next{types::EntityId::LEFT_GOBLIN, types::TurnType::FAN, 0};
    auto deltaRequest = aiTools::getNextFanTurn(state, next);
    EXPECT_THAT(deltaRequest.getDeltaType(), testing::AnyOf(types::DeltaType::SKIP, types::DeltaType::GOBLIN_SHOCK));
    if(deltaRequest.getDeltaType() == types::DeltaType::GOBLIN_SHOCK) {
        std::optional<types::EntityId> entityId = deltaRequest.getPassiveEntity();
        if(entityId.has_value()) {
            auto player = env->getPlayerById(entityId.value());
            gameController::RangedAttack rangedAttack {env, env->team2, player};
            EXPECT_TRUE(rangedAttack.isPossible());
        }
    }
}

TEST(ai_test, getNextFanTurn1){
    using namespace communication::messages;
    auto env = setup::createEnv();
    aiTools::State state;
    state.env = env;
    broadcast::Next next{types::EntityId::LEFT_WOMBAT, types::TurnType::FAN, 0};
    auto deltaRequest = aiTools::getNextFanTurn(state, next);
    EXPECT_THAT(deltaRequest.getDeltaType(), testing::AnyOf (types::DeltaType::SKIP, types::DeltaType::WOMBAT_POO));
}

TEST(ai_test, getNextFanTurn2){
    using namespace communication::messages;
    auto env = setup::createEnv();
    aiTools::State state;
    state.env = env;
    broadcast::Next next{types::EntityId::LEFT_TROLL, types::TurnType::FAN, 0};
    auto deltaRequest = aiTools::getNextFanTurn(state, next);
    EXPECT_THAT(deltaRequest.getDeltaType(), testing::AnyOf(types::DeltaType::SKIP, types::DeltaType::TROLL_ROAR));
    if(deltaRequest.getDeltaType() == types::DeltaType::TROLL_ROAR) {
        gameController::Impulse impulse {env, env->team2};
        EXPECT_TRUE(impulse.isPossible());
    }
}

TEST(ai_test, getNextFanTurn3){
    using namespace communication::messages;
    auto env = setup::createEnv();
    aiTools::State state;
    state.env = env;
    env->snitch->exists = true;
    broadcast::Next next{types::EntityId::LEFT_ELF, types::TurnType::FAN, 0};
    auto deltaRequest = aiTools::getNextFanTurn(state, next);
    EXPECT_THAT(deltaRequest.getDeltaType(), testing::AnyOf(types::DeltaType::SKIP, types::DeltaType::ELF_TELEPORTATION));
    if(deltaRequest.getDeltaType() == types::DeltaType::ELF_TELEPORTATION) {
        auto entityId = deltaRequest.getPassiveEntity();
        if(entityId.has_value()) {
            auto player = env->getPlayerById(entityId.value());
            gameController::Teleport teleport {env, env->team2, player};
            EXPECT_TRUE(teleport.isPossible());
            EXPECT_TRUE(env->getTeam(player)->side == gameModel::TeamSide::RIGHT);
        }
    }
}

TEST(ai_test, getNextFanTurn4){
    using namespace communication::messages;
    auto env = setup::createEnv();
    aiTools::State state;
    state.env = env;
    env->snitch->exists = true;
    broadcast::Next next{types::EntityId::LEFT_NIFFLER, types::TurnType::FAN, 0};
    auto deltaRequest = aiTools::getNextFanTurn(state, next);
    EXPECT_THAT(deltaRequest.getDeltaType(), testing::AnyOf(types::DeltaType::SKIP, types::DeltaType::SNITCH_SNATCH));
    if(deltaRequest.getDeltaType() == communication::messages::types::DeltaType::SNITCH_SNATCH) {
        gameController::SnitchPush snitchPush {env, env->team2};
        EXPECT_TRUE(snitchPush.isPossible());
    }
}
