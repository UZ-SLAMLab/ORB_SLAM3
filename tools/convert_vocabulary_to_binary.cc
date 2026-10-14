/**
* Converts an ORB vocabulary from DBoW2's text format (Vocabulary/ORBvoc.txt)
* to the binary format of TemplatedVocabulary::saveToBinaryFile(), which
* ORB_SLAM3::System loads in well under a second instead of several seconds,
* when given a vocabulary path ending in ".bin".
*
* The text file's MD5 is stored in the binary file, and System uses it as the
* vocabulary checksum saved in and checked against atlases (.osa), so atlases
* saved with either vocabulary file load with the other.
*
* After writing, the binary file is loaded back and saved again, and the two
* files are compared byte for byte, to check that nothing was lost.
*
* Usage: ./convert_vocabulary_to_binary Vocabulary/ORBvoc.txt Vocabulary/ORBvoc.bin
*/

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

#include <openssl/md5.h>

#include "ORBVocabulary.h"

using namespace std;

// Same digest as System::CalculateCheckSum(file, TEXT_FILE): MD5 of the
// file's bytes, as lowercase hex.
static string Md5OfFile(const string &filename)
{
    ifstream f(filename.c_str(), ios::in | ios::binary);
    if(!f.is_open())
        return "";
    MD5_CTX md5Context;
    MD5_Init(&md5Context);
    char buffer[1 << 16];
    while(f.read(buffer, sizeof(buffer)) || f.gcount() > 0)
        MD5_Update(&md5Context, buffer, f.gcount());
    unsigned char c[MD5_DIGEST_LENGTH];
    MD5_Final(c, &md5Context);
    string checksum;
    for(int i = 0; i < MD5_DIGEST_LENGTH; i++)
    {
        char aux[3];
        snprintf(aux, sizeof(aux), "%02x", c[i]);
        checksum += aux;
    }
    return checksum;
}

static string ReadFile(const string &filename)
{
    ifstream f(filename.c_str(), ios::in | ios::binary);
    return string(istreambuf_iterator<char>(f), istreambuf_iterator<char>());
}

static double SecondsSince(const chrono::steady_clock::time_point &t0)
{
    return chrono::duration<double>(chrono::steady_clock::now() - t0).count();
}

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << "Usage: " << argv[0] << " path_to_ORBvoc.txt path_to_output.bin" << endl;
        return 1;
    }
    const string txtFile = argv[1];
    const string binFile = argv[2];

    const string checksum = Md5OfFile(txtFile);
    if(checksum.empty())
    {
        cerr << "ERROR: can't open " << txtFile << endl;
        return 1;
    }

    auto t0 = chrono::steady_clock::now();
    ORB_SLAM3::ORBVocabulary voc;
    if(!voc.loadFromTextFile(txtFile))
    {
        cerr << "ERROR: can't load text vocabulary " << txtFile << endl;
        return 1;
    }
    cout << "Loaded " << txtFile << " (" << voc.size() << " words, MD5 " << checksum << ") in "
         << SecondsSince(t0) << " s" << endl;

    if(!voc.saveToBinaryFile(binFile, checksum))
    {
        cerr << "ERROR: can't write " << binFile << endl;
        return 1;
    }

    t0 = chrono::steady_clock::now();
    ORB_SLAM3::ORBVocabulary reloaded;
    string reloadedChecksum;
    if(!reloaded.loadFromBinaryFile(binFile, &reloadedChecksum))
    {
        cerr << "ERROR: can't load back " << binFile << endl;
        return 1;
    }
    cout << "Loaded back " << binFile << " in " << SecondsSince(t0) << " s" << endl;

    const string roundTripFile = binFile + ".roundtrip";
    const bool bSaved = reloaded.saveToBinaryFile(roundTripFile, reloadedChecksum);
    const bool bIdentical = bSaved && ReadFile(binFile) == ReadFile(roundTripFile);
    remove(roundTripFile.c_str());
    if(!bIdentical || reloaded.size() != voc.size() || reloadedChecksum != checksum)
    {
        cerr << "ERROR: " << binFile << " doesn't load back identically" << endl;
        remove(binFile.c_str());
        return 1;
    }
    cout << "Wrote " << binFile << " (verified)" << endl;
    return 0;
}
