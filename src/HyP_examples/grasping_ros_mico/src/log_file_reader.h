/* 
 * File:   log_file_reader.h
 * Author: neha
 *
 * Created on November 12, 2016, 4:48 PM
 */

#ifndef LOG_FILE_READER_H
#define	LOG_FILE_READER_H

#include <fstream>
#include "history_with_reward.h"

class log_file_reader {
public:
    log_file_reader() {};
    log_file_reader(std::string logfilename) : logFileName(logfilename) {};
    virtual ~log_file_reader() {};
    
    std::string logFileName;
    
    HistoryWithReward getHistoryWithReward();
    void testPythonCall();

};

#endif	/* LOG_FILE_READER_H */

