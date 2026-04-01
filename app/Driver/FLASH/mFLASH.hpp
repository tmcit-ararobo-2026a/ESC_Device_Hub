
#pragma once
#include "mFLASH_data_template.hpp"

class mFLASH_Class : mFLASH_data_template_Class{
    public:
        State Init();
        State Write();
        State Read();
};

extern mFLASH_Class mFLASH;
