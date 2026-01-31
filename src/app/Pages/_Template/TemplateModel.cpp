#include "TemplateModel.h"

using namespace Page;

uint32_t TemplateModel::GetData()
{
    return lv_tick_get();
}


void TemplateModel::Init()
{
    account = new Account("TemplateModel", AccountSystem::Broker(), 0, this);
    account->Subscribe("BotStatus");
    // account->Subscribe("Power");
    // account->Subscribe("Storage");
}

void TemplateModel::Deinit()
{
    if (account)
    {
        delete account;
        account = nullptr;
    }
}

void TemplateModel::GetBotInfo(int *status)
{

    AccountSystem::BotStatusInfo bot_status;
    if (!account->Pull("BotStatus", &bot_status, sizeof(bot_status))) {
        *status = bot_status.running_mode;
    } else {
        status = 0;
    }
    
}
