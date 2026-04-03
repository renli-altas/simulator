#pragma once

#include "AbstractMmu.h"
#include "config.h"

class SimContext; // Forward declaration
class RealLsu;

class SimpleMmu : public AbstractMmu {
private:
    SimContext *ctx;
    RealLsu *lsu = nullptr;

public:
    SimpleMmu(SimContext *ctx, RealLsu *lsu = nullptr);

    Result translate(uint32_t &p_addr, uint32_t v_addr, uint32_t type,
                     CsrStatusIO *status) override;
};
