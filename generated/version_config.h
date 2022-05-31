#ifndef VERSION_CONFIG_H
#define VERSION_CONFIG_H

#include <string>

// define your version_libinterface
#define ELECTROSENSE_VERSION "1.3.8"

// alternatively you could add your global method getElectrosenseVersion here

std::string getElectrosenseVersion()
{
    return "1.3.8-1";
}

std::string getElectrosenseTimeCompilation()
{
    return "2022-05-31 14:26";
}


#endif // VERSION_CONFIG_H
