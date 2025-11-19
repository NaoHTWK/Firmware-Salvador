#include <stdio.h>
#include <tts.h>

#include <iomanip>
#include <iostream>
#include <sstream>


void say(const std::string& txt) {
    printf("Say: '%s'\n", txt.c_str());
    std::string cmd = "/home/booster/etc/say -t \"" + txt + "\"&";
    std::system(cmd.c_str());
}

void say(const std::string& txt, float speed_stretch_factor) {
    printf("Say: '%s'\n", txt.c_str());
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << speed_stretch_factor;
    std::string s = stream.str();

    std::string cmd = "/home/booster/etc/say --setf duration_stretch=" + s +
                      " --setf int_f0_target_mean=200 -t \"" + txt + "\"&";
    std::cout << cmd << std::endl;
    std::system(cmd.c_str());
}
