//
// Created by tim on 16.06.19.
//

#ifndef KI_SETUP_H
#define KI_SETUP_H

#include <SopraGameLogic/GameModel.h>

namespace setup{
    auto createEnv() -> std::shared_ptr<gameModel::Environment>;
    auto createEnv(const gameModel::Config &config) -> std::shared_ptr<gameModel::Environment>;
    auto createSymmetricEnv() -> std::shared_ptr<gameModel::Environment>;
}

#endif //KI_SETUP_H
