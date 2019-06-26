//
// Created by tim on 16.06.19.
//

#include "setup.h"

auto setup::createEnv() -> std::shared_ptr<gameModel::Environment> {
    return createEnv({0, {}, {}, {}});
}

auto setup::createEnv(const gameModel::Config &config) -> std::shared_ptr<gameModel::Environment> {
    using ID = communication::messages::types::EntityId;
    gameModel::Chaser c1({2, 10}, {}, ID::LEFT_CHASER1);
    gameModel::Chaser c2({8, 5}, {}, ID::LEFT_CHASER2);
    gameModel::Chaser c3({10, 7}, {}, ID::LEFT_CHASER3);
    gameModel::Chaser c4({6, 1}, {}, ID::RIGHT_CHASER1);
    gameModel::Chaser c5({9, 9}, {}, ID::RIGHT_CHASER2);
    gameModel::Chaser c6({7, 3}, {}, ID::RIGHT_CHASER3);

    gameModel::Beater b1({1, 3}, {}, ID::LEFT_BEATER1);
    gameModel::Beater b2({3, 0}, {}, ID::LEFT_BEATER2);
    gameModel::Beater b3({0, 6}, {}, ID::RIGHT_BEATER1);
    gameModel::Beater b4({4, 2}, {}, ID::RIGHT_BEATER2);

    gameModel::Seeker s1({5, 4}, {}, ID::LEFT_SEEKER);
    gameModel::Seeker s2({11, 8}, {}, ID::RIGHT_SEEKER);

    gameModel::Keeper k1({12, 11}, {}, ID::LEFT_KEEPER);
    gameModel::Keeper k2({13, 12}, {}, ID::RIGHT_KEEPER);

    gameModel::Fanblock f(1, 2, 2, 1, 1);

    auto t1 = std::make_shared<gameModel::Team>(s1, k1, std::array<gameModel::Beater, 2>{b1, b2}, std::array<gameModel::Chaser, 3>{c1, c2, c3}, 0, f, gameModel::TeamSide::LEFT);
    auto t2 = std::make_shared<gameModel::Team>(s2, k2, std::array<gameModel::Beater, 2>{b3, b4}, std::array<gameModel::Chaser, 3>{c4, c5, c6}, 0, f, gameModel::TeamSide::RIGHT);

    return std::make_shared<gameModel::Environment>(config, t1, t2);
}

auto setup::createSymmetricEnv() -> std::shared_ptr<gameModel::Environment> {

    auto env = createEnv();

    env->team1->keeper->position = {6, 7};
    env->team2->keeper->position = {10, 7};

    env->team1->seeker->position = {6, 6};
    env->team2->seeker->position = {10, 6};

    env->team1->beaters[0]->position = {6, 4};
    env->team1->beaters[1]->position = {6, 5};
    env->team2->beaters[0]->position = {10, 4};
    env->team2->beaters[1]->position = {10, 5};

    env->team1->chasers[0]->position = {6, 0};
    env->team1->chasers[1]->position = {6, 1};
    env->team1->chasers[2]->position = {6, 2};
    env->team2->chasers[0]->position = {10, 0};
    env->team2->chasers[1]->position = {10, 1};
    env->team2->chasers[2]->position = {10, 2};

    return env;
}


