#ifndef DATA_PROCESSOR_HPP
#define DATA_PROCESSOR_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

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
} // namespace DataProcessor

#endif