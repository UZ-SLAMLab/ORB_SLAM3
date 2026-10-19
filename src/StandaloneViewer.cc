#include "StandaloneViewer.h"
#include <pangolin/pangolin.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <openssl/md5.h>
#include <thread>

namespace ORB_SLAM3 {
    StandaloneViewer::StandaloneViewer(const string &pStrLoadAtlasFromFile,
                                       const string &strVocFile,
                                       const string &strSettingsFile) : 
                                        mStrLoadAtlasFromFile(pStrLoadAtlasFromFile),
                                        mStrVocabularyFilePath(strVocFile), mptViewer(nullptr), 
                                        mpViewer(nullptr), isSaving(false)
    {
        cout << endl
                << "=====================================================================" << endl
                << "ORB-SLAM3 Standalone Viewer." << endl
                << "INITIALIZING..." << endl
                << "=====================================================================" << endl
                << endl;

        if (mStrLoadAtlasFromFile.empty()) {
            cout << endl
                    << "ERROR: No input file was provided..." << endl;

            exit(-1);
        }

        // Load ORB Vocabulary
        cout << endl
                << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(mStrVocabularyFilePath);
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << mStrVocabularyFilePath << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl
                << endl;

        // Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        cout << "Load File" << endl;

        // Load the file with an earlier session
        cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if (!isRead) {
            cout << "Error loading the file, please try with other session file or vocabulary file" << endl;
            exit(-1);
        }

        mvpMaps = mpAtlas->GetAllMaps();

        // set active map to the one with the most keyframes
        if (!mvpMaps.empty()) {
            int maxKFs = -1;
            int idx = 0;

            for (Map* pMap : mvpMaps) {
                if ((int)pMap->GetAllKeyFrames().size() > maxKFs) {
                    maxKFs = (int)pMap->GetAllKeyFrames().size();
                    pCurrentMapIdx = idx;
                    pCurrentMap = pMap;
                }
                idx++;
            }

            if (pCurrentMap) {
                cout << "Setting map with ID " << pCurrentMap->GetId() << " as active map." << endl;
                mpAtlas->ChangeMap(pCurrentMap);
            } else {
                cout << "No maps found in the atlas." << endl;
            }
        }
        
        mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, nullptr);

        mpViewer = new Viewer(this, mpMapDrawer, strSettingsFile);
        mptViewer = new std::thread(&Viewer::Run, mpViewer);

        // Fix verbosity
        Verbose::SetTh(Verbose::VERBOSITY_QUIET);
    }

    Map* StandaloneViewer::GetCurrentMap() {
        return pCurrentMap;
    }

    int StandaloneViewer::GetCurrentMapIdx() {
        return pCurrentMapIdx;
    }

    int StandaloneViewer::GetNumberOfMaps() {
        return (int)mvpMaps.size();
    }

    void StandaloneViewer::ChangeMap(int mapIdx) {
        Map *pMap = mvpMaps[mapIdx];

        if (pMap) {
            mpAtlas->ChangeMap(pMap);
            pCurrentMap = pMap;
            pCurrentMapIdx = mapIdx;
        }
    }

    void StandaloneViewer::DeleteSubmap() {
        if (mvpMaps.size() <= 1) {
            cout << "[StandaloneViewer] Warning: Cannot delete the last remaining submap in the Atlas!" << endl;
            return;
        }
    
        if (pCurrentMapIdx < 0 || pCurrentMapIdx >= (int)mvpMaps.size())
            return;

        badMaps.push_back(pCurrentMap);
        badMapIndxs.push_back(pCurrentMapIdx);
        mvpMaps.erase(mvpMaps.begin() + pCurrentMapIdx);

        int newMapIdx = pCurrentMapIdx - 1;
        if (newMapIdx >= 0) ChangeMap(newMapIdx);       // mvpMaps.size() must have been > 1 before deletion, bc index was >= 1 --> deleted element was not 1st
        else ChangeMap(0);                              // newMapIdx < 0 --> 1st element was deleted 
    }

    void StandaloneViewer::SaveChanges() {
        isSaving = true;
        while (!badMaps.empty()) {
            Map *pMap = badMaps.front();

            mpAtlas->SetMapBad(pMap);

            badMaps.erase(badMaps.begin());
            badMapIndxs.erase(badMapIndxs.begin());
        }

        mpAtlas->RemoveBadMaps();

        if (!mStrLoadAtlasFromFile.empty())
        {
            Verbose::PrintMess("Atlas saving to file " + mStrLoadAtlasFromFile, Verbose::VERBOSITY_NORMAL);
            SaveAtlas(FileType::BINARY_FILE);
        }

        isSaving = false;
    }

    bool StandaloneViewer::IsSaving() {
        return isSaving;
    }

    void StandaloneViewer::UndoChanges() {
        while (!badMaps.empty()) {
            Map *map = badMaps.back();
            int idx = badMapIndxs.back();

            mvpMaps.insert(mvpMaps.begin() + idx, map);
            badMaps.pop_back();
            badMapIndxs.pop_back();
        }
    }

    bool StandaloneViewer::LoadAtlas(int type) {
        string strFileVoc, strVocChecksum;
        bool isRead = false;

        string pathLoadFileName = mStrLoadAtlasFromFile;
        
        cout << "load file name: " << pathLoadFileName << endl;

        if (type == BINARY_FILE) // File binary
        {
            cout << "Starting to read the save binary file" << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good()) {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::binary_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save binary file" << endl;
            isRead = true;
        }
        else // File text
        {
            cout << "Please select a BINARY file to load" << endl;
        } 

        if (isRead) {
            // Check if the vocabulary is the same
            string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

            if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
                cout << "The vocabulary load isn't the same which the load session was created " << endl;
                cout << "-Vocabulary name: " << strFileVoc << endl;
                return false; // Both are differents
            }

            mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
            mpAtlas->SetORBVocabulary(mpVocabulary);
            mpAtlas->PostLoad();

            return true;
        }
        return false;
    }

    void StandaloneViewer::SaveAtlas(int type)
    {
        if (!mStrLoadAtlasFromFile.empty())
        {
            // clock_t start = clock();

            // Save the current session
            mpAtlas->PreSave();

            string pathSaveFileName = mStrLoadAtlasFromFile;

            string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
            std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
            string strVocabularyName = mStrVocabularyFilePath.substr(found + 1);

            if (type == TEXT_FILE) // File text
            {
                cout << "Starting to write the save text file " << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::text_oarchive oa(ofs);

                oa << strVocabularyName;
                oa << strVocabularyChecksum;
                oa << mpAtlas;
                cout << "End to write the save text file" << endl;

                
            }
            else if (type == BINARY_FILE) // File binary
            {
                cout << "Writing into binary file ..." << endl;
                std::remove(pathSaveFileName.c_str());

                {
                    std::ofstream ofs(pathSaveFileName, std::ios::binary);
                    if (!ofs.is_open()) {
                        cerr << "\n[ERROR] File couldn't be written. Please check permissions!\n" << endl;
                        return;
                    }

                    boost::archive::binary_oarchive oa(ofs);
                    oa << strVocabularyName;
                    oa << strVocabularyChecksum;
                    oa << mpAtlas;
                }

                // compatibility fix:
                // On tested devices (for example Meta Quest 3): there was a lower version of boost installed.
                // On the Quest 3, Boost 1.83 (or lower) was installed. In the binary, this was called version 19.
                // When we edit the file on another system however, it writes the boost version installed on said
                // system into the file. When the two versions differ, this may lead to an error.
                // The boost version is written to Byte number 30 of the binary file.
                // So we manually rewrite Byte 30 to say that the file was written with boost version 19.
                std::fstream fs(pathSaveFileName, std::ios::in | std::ios::out | std::ios::binary);

                if (fs.is_open()) {
                    fs.seekp(30);   // byte 30
                    char v = 19;    // version 19
                    fs.write(&v, 1);
                    fs.close();
                }

                cout << "File saved!" << endl;
            }
        }
    }

    void StandaloneViewer::Shutdown()
    {
        cout << "Waiting for Pangolin-Viewer to finish ..." << endl;

        if (mpViewer)
        {
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished())
                usleep(5000);
        }

        if (mptViewer && mptViewer->joinable()) {
            mptViewer->join();
        }
    }

    string StandaloneViewer::CalculateCheckSum(string filename, int type) {
        string checksum = "";

        unsigned char c[MD5_DIGEST_LENGTH];

        std::ios_base::openmode flags = std::ios::in;
        if (type == BINARY_FILE) // Binary file
            flags = std::ios::in | std::ios::binary;

        ifstream f(filename.c_str(), flags);
        if (!f.is_open()) {
            cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
            return checksum;
        }

        MD5_CTX md5Context;
        char buffer[1024];

        MD5_Init(&md5Context);
        while (int count = f.readsome(buffer, sizeof(buffer))) {
            MD5_Update(&md5Context, buffer, count);
        }

        f.close();

        MD5_Final(c, &md5Context);

        for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
            char aux[10];
            sprintf(aux, "%02x", c[i]);
            checksum = checksum + aux;
        }

        return checksum;
    }
}
