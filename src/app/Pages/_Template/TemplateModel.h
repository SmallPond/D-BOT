#ifndef __TEMPLATE_MODEL_H
#define __TEMPLATE_MODEL_H

#include "app/Accounts/Account_Master.h"
#include "lvgl.h"

namespace Page
{

class TemplateModel
{
public:
    uint32_t TickSave;
    uint32_t GetData();

    void Init();
    void Deinit();
    void GetBotInfo(int *status);
private:
    Account* account;
};

}

#endif
