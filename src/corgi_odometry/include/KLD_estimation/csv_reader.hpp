#ifndef DATA_PROCESSOR_HPP
#define DATA_PROCESSOR_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <sys/stat.h>

namespace DataProcessor
{
    using namespace std;
    struct DataRow{
        public:
        map<int, float> data;
    };
    struct DataFrame{
        public:
        map<string, int> columns;
        map<int, DataRow> data;
        int row = 0;

        friend ostream &operator<<(ostream &s, DataFrame df)
        {
            for (auto col : df.columns){
                s << col.first << "\t";
            }
            s << "\n";
            for (auto iter_row : df.data)
            {
                for (auto iter_col : iter_row.second.data)
                {
                    s << iter_col.second << "\t";
                }
                s << "\n";
            }
            return s; 
        }
        float iloc(string col, int row)
        {
            return data[row].data[columns[col]];
        }
    };

    DataFrame read_csv(string filename, bool header=true)
    {
        DataFrame file;
        ifstream RawFile(filename, ios::in);
        if (!RawFile)
        {
            cout << "No such file name:\t" << filename << endl;
            exit(1);
        }   
        string line;
        if (header)
        {
            getline(RawFile, line);
            istringstream delim(line);
            string token;
            int c = 0;
            while (getline(delim, token, ',')) 
            {
                file.columns[token] = c;
                c ++;
            }
        }
        int r = 0;
        while (getline(RawFile, line))
        {
            istringstream delim(line);
            string token;
            int c = 0;
            while (getline(delim, token, ',')) 
            {
                file.data[r].data[c] = stod(token);
                c ++;
            }
            r++;
        }
        file.row = r;
        return file;
    }

    void write_csv(Eigen::MatrixXf data, string filename, vector<string> header)
    {
        ofstream outputFile(filename, ios::out);
        
        if (outputFile.is_open()) {
            for (auto col: header){
                outputFile << col << ",";
            }
            outputFile << "\n";
            for (int i = 0; i < data.rows(); ++i) {
                for (int j = 0; j < data.cols(); ++j) {
                    outputFile << data(i, j);
                    if (j < data.cols() - 1) {
                        outputFile << ",";
                    }
                }
                outputFile << endl;
            }
            outputFile.close();

            cout << "Write Success" << endl;
        } else {
            cerr << "Cannot Open File" << endl;
        }
    }
    class CsvLogger {
    public:
        std::ofstream outFile;

        bool init = false;

        CsvLogger() = default;
        
        // Destructor makes sure the file is closed.
        ~CsvLogger() {
            if (outFile.is_open()) {
                outFile.close();
            }
        }
        
        /**
         * @brief Initialize the CSV file by opening it and writing the header.
         * 
         * @param filename The name (and path) of the file to create.
         * @param headers A vector of strings representing header names (each column name).
         * @return true if the file was successfully opened and the header was written.
         * @return false otherwise.
         */
        bool initCSV(const std::string &filename, const std::vector<std::string>& headers) {
            this->filename = filename;
            // Open file in output mode (this overwrites any existing file with the same name)
            outFile.open(filename, std::ios::out);
            if (!outFile.is_open()) {
                std::cerr << "Error opening file: " << filename << std::endl;
                return false;
            }
            // Write headers: comma-separated, no trailing comma.
            for (size_t i = 0; i < headers.size(); ++i) {
                outFile << headers[i];
                if (i != headers.size() - 1) {
                    outFile << ",";
                }
            }
            outFile << "\n";
            outFile.flush();  // make sure header is written immediately
            init = true;
            return true;
        }
        
        /**
         * @brief Log a new state (an Eigen::VectorXf) by appending it to the CSV file.
         * 
         * @param state The state vector to log.
         * @return true if the state was successfully logged.
         * @return false if the file is not open.
         */
        bool logState(const Eigen::VectorXf &state) {
            if (!outFile.is_open()) {
                std::cerr << "File not open. Please call initCSV() first." << std::endl;
                return false;
            }
            // Write each element separated by commas.
            for (int i = 0; i < state.size(); ++i) {
                outFile << state(i);
                if (i < state.size() - 1)
                    outFile << ",";
            }
            outFile << "\n";
            return true;
        }
        
        /**
         * @brief Finalize the CSV file by flushing and closing the stream.
         */
        void finalizeCSV() {
            if (outFile.is_open()) {
                outFile.flush();
                outFile.close();
            }
        }

        bool file_exists(const std::string &filename) {
            struct stat buffer;
            return (stat(filename.c_str(), &buffer) == 0);
        }
        
    private:
        std::string filename;
    };
} // namespace DataProcessor

#endif