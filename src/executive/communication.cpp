#include "communication.h"
#include <sys/wait.h>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <stdlib.h>


using namespace std;

Communication::Communication(const std::string& pathToExecutable, bool bindStdin, bool bindStdout, bool bindStderr, bool sendPipetoChild)
{
    set(pathToExecutable,bindStdin,bindStdout,bindStderr,sendPipetoChild);
}

Communication::~Communication()
{
    if (pid != 1)
    {
        close(sendp[1]);
        close(getp[0]);
        kill(pid, SIGKILL);
        waitpid(pid, &status, 0);
    }
    else
    {
        exit(1);
    }
}

void Communication::set(const std::string& pathToExecutable, bool bindStdin, bool bindStdout, bool bindStderr, bool sendPipetoChild)
{
    pipe(sendp);
    pipe(getp);
    pid = fork();
    if (pid == 0) //child
    {
        if (sendPipetoChild)
        {
            
            close(sendp[1]);
            close(getp[0]);
            execl(pathToExecutable.c_str(),pathToExecutable.c_str(),to_string(sendp[0]).c_str(),to_string(getp[1]).c_str(), (char *)nullptr);
        }
        else
        {
            if (bindStdin)
                dup2(sendp[0], fileno(stdin));
            if (bindStdout)
                dup2(getp[1], fileno(stdout));
            if (bindStderr)
                dup2(getp[1], fileno(stderr));

            close(sendp[0]);
            close(sendp[1]);
            close(getp[0]);
            close(getp[1]);
            execl(pathToExecutable.c_str(),pathToExecutable.c_str(), (char *)NULL);
            cerr << "execl failed: " << strerror(errno) << endl;
        }
    }
    else
    {
        close(sendp[0]);
        close(getp[1]);

        readstream = fdopen(getWpipe(), "r");
//        cerr << "Setting readstream to " << readstream << endl;
//        lk.unlock();
    }
}

void Communication::set(const std::string& pathToExecutable, bool bindStdin, bool bindStdout, bool bindStderr, bool sendPipetoChild, const string& s)
{
//    unique_lock<mutex> lk(mtx);
//    lk.lock();
    pipe(sendp);
    pipe(getp);
    pid = fork();
    if (pid == 0) //child
    {
//        lk.unlock();
        if (sendPipetoChild)
        {
            
            close(sendp[1]);
            close(getp[0]);
            execl(pathToExecutable.c_str(),pathToExecutable.c_str(),to_string(sendp[0]).c_str(),to_string(getp[1]).c_str(),s.c_str(), (char *)nullptr);
        }
        else
        {
            if (bindStdin)
                dup2(sendp[0], fileno(stdin));
            if (bindStdout)
                dup2(getp[1], fileno(stdout));
            if (bindStderr)
                dup2(getp[1], fileno(stderr));

            close(sendp[0]);
            close(sendp[1]);
            close(getp[0]);
            close(getp[1]);
            execl(pathToExecutable.c_str(),pathToExecutable.c_str(), (char *)nullptr);
        }
        
    }
    else
    {
        close(sendp[0]);
        close(getp[1]);

        readstream = fdopen(getWpipe(), "r");
//        cerr << "Setting readstream to " << readstream << endl;
//        lk.unlock();
    }
}

void Communication::cwrite(const std::string& s)
{
    write(sendp[1], s.c_str(), s.length());
    write(sendp[1], "\n", 1); //delete if need
}

void Communication::cwrite(char s[])
{
    write(sendp[1], s, strlen(s));
    write(sendp[1], "\n", 1); //delete if need
}

void Communication::cwrite(const char s[])
{
    write(sendp[1], s, strlen(s));
    write(sendp[1], "\n", 1);
}

void Communication::cread(char receive[], int length) const
{
    read(getp[0], receive, length);
}

void Communication::readAll(char receive[])
{
//    unique_lock<mutex> lk(mtx);
//    lk.lock();
//    cerr << "readstream is a pointer with the value " << readstream << endl;
    auto result = fgets(receive, 1024, readstream);
//    if (result != nullptr) {
//        cerr << "Read string: *** \n" << result << "\n***" << endl;
//    } else {
//        cerr << "found nullptr" << endl;
//    }
//    lk.unlock();
}

pid_t Communication::getPid() {
    return pid;
}
