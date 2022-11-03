#pragma once

struct DictonaryElement {
    unsigned int can_ID;
    std::string name;
};

class Dictonary {
    private:
        void setName(unsigned int can_ID, std::string name);
        std::string getName(unsigned int can_ID);
    public:

};