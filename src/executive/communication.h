/*
* open two way pipe for desire program.
* Chao Chi Cheng
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include<unistd.h>
//#include<sys/wait.h>
//#include<signal.h>
//#include<stdlib.h>
#include <string>
//#include<stdio.h>
#include <mutex>

class Communication
{

  public:
    Communication(const std::string& pathToExecutable,bool bindStdin, bool bindStdout, bool bindStderr, bool sendPipetoChild);

    Communication()
    {
    }

    ~Communication();

    void set(const std::string& pathToExecutable,bool bindStdin, bool bindStdout, bool bindStderr, bool sendPipetoChild);

    void set(const std::string& pathToExecutable,bool bindStdin, bool bindStdout, bool bindStderr, bool sendPipetoChild,const std::string& s);

    void cwrite(const std::string& s);

    void cwrite(char s[]);

    void cwrite(const char s[]);

    void cread(char receive[], int length) const;

    void readAll(char receive[]);

    int getWpipe()
    {
      return getp[0];
    };

    pid_t getPid();


  private:
    pid_t pid = 0;
    int getp[2];
    int sendp[2];
    int status;

    std::mutex mtx;
    FILE* readstream;
};

#endif
