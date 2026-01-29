#ifndef TERMINATION_HANDLER
#define TERMINATION_HANDLER
#include <csignal>
#include <functional>
#include <vector>;

class TerminationHandler
{
    private:
    std::vector<std::function<void()>> _callbacks;
    static TerminationHandler* _instance;

    static void Handler(int signal)
    {
        for(auto callback : _instance->_callbacks)
            callback();
    }

    public:
    static void RegisterCallback(std::function<void()> c)
    {
        _instance->_callbacks.push_back(c);
    }
    TerminationHandler()
    {
        _instance = this;
        struct sigaction sigIntAction{};
        sigIntAction.sa_handler = Handler;
        sigemptyset(&sigIntAction.sa_mask);
        sigIntAction.sa_flags = 0;
        sigaction(SIGINT, &sigIntAction, nullptr);
    }
};

TerminationHandler* TerminationHandler::_instance;

#endif