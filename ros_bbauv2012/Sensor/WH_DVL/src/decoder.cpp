//decoder.cpp

#include "decoder.h"

Decoder::Decoder() {
}

Decoder::~Decoder() {
}

void Decoder::DecodeBBensemble() {
    //cout << "Decoder::DecodeBBensemble is called" << endl;
    int offset;
    string ID, first_half, last_half;

    if(checkRcvEnsemble() != 0) return;
    solveHeader();

    for (int i=0; i < header.NDataTypes; i++) {
        offset = header.Offset[i];
        ID = rcvStr.substr(offset*2, 4);
        first_half = ID.substr(0, 2);
        last_half = ID.substr(2, 2);
        ID = last_half + first_half;

        if (ID == FLdrSelected) solveFixLeader(offset);
        else if (ID == VLdrSelected) solveVarLeader(offset);
        else if (ID == BotSelected) solveBotTrack(offset);
        else cout << "ID not recognized: " << ID << endl;
    }
} //end decodeBBensemble()

void Decoder::solveHeader() {
    //cout << "Decoder::Solving header" << endl;
    
    header.ChecksumOffset = invByteHexToDec(rcvStr.substr(4,4));
    header.NDataTypes = invByteHexToDec(rcvStr.substr(10,2));
    for (int i=0; i<header.NDataTypes; i++) {
        header.Offset[i] = invByteHexToDec(rcvStr.substr(12+4*i,4));
    }
} //end solveHeader()

void Decoder::solveFixLeader(ushort offset) {
    //cout << "Decoder::Solving fix leader" << endl;

    fixLeader.CPUFirmware.Version     = invByteHexToDec(rcvStr.substr(offset*2 + 4, 2));
    fixLeader.CPUFirmware.Revision    = invByteHexToDec(rcvStr.substr(offset*2 + 6, 2));
    fixLeader.Configuration           = invByteHexToDec(rcvStr.substr(offset*2 + 8, 4));
    fixLeader.DummyDataFlag           = invByteHexToDec(rcvStr.substr(offset*2 + 12, 2));
    fixLeader.Lag                     = invByteHexToDec(rcvStr.substr(offset*2 + 14, 2));
    fixLeader.NBeams                  = invByteHexToDec(rcvStr.substr(offset*2 + 16, 2));
    fixLeader.NBins                   = invByteHexToDec(rcvStr.substr(offset*2 + 18, 2));
    if (fixLeader.NBins > 128) fixLeader.NBins = 32;
    fixLeader.PingsPerEnsemble        = invByteHexToDec(rcvStr.substr(offset*2 + 20, 4));
    fixLeader.BinLength               = invByteHexToDec(rcvStr.substr(offset*2 + 24, 4));
    fixLeader.BlankAfterTransmit      = invByteHexToDec(rcvStr.substr(offset*2 + 28, 4));
    fixLeader.SignalProMode           = invByteHexToDec(rcvStr.substr(offset*2 + 32, 2));
    fixLeader.PctCorrelationLow       = invByteHexToDec(rcvStr.substr(offset*2 + 34, 2));
    fixLeader.NCodeRepetitions        = invByteHexToDec(rcvStr.substr(offset*2 + 36, 2));
    fixLeader.PctGoodMin              = invByteHexToDec(rcvStr.substr(offset*2 + 38, 2));
    fixLeader.ErrVelocityThres        = invByteHexToDec(rcvStr.substr(offset*2 + 40, 4));
    fixLeader.TimeBetweenPings.Min    = invByteHexToDec(rcvStr.substr(offset*2 + 44, 2));
    fixLeader.TimeBetweenPings.Sec    = invByteHexToDec(rcvStr.substr(offset*2 + 46, 2));
    fixLeader.TimeBetweenPings.Sec100 = invByteHexToDec(rcvStr.substr(offset*2 + 48, 2));
    fixLeader.CoordSystemParms        = invByteHexToDec(rcvStr.substr(offset*2 + 50, 2));
    fixLeader.HeadingAlignment        = invByteHexToDec(rcvStr.substr(offset*2 + 52, 4));
    fixLeader.HeadingBias             = invByteHexToDec(rcvStr.substr(offset*2 + 56, 4));
    fixLeader.SensorSource            = invByteHexToDec(rcvStr.substr(offset*2 + 60, 2));
    fixLeader.AvailableSensors        = invByteHexToDec(rcvStr.substr(offset*2 + 62, 2));
    fixLeader.Bin1Dist                = invByteHexToDec(rcvStr.substr(offset*2 + 64, 4));
    fixLeader.PulseLength             = invByteHexToDec(rcvStr.substr(offset*2 + 68, 4));
    fixLeader.StartDepthCell          = invByteHexToDec(rcvStr.substr(offset*2 + 72, 2));
    fixLeader.EndDepthCell            = invByteHexToDec(rcvStr.substr(offset*2 + 74, 2));
    fixLeader.FalseTargetThres        = invByteHexToDec(rcvStr.substr(offset*2 + 76, 2));
    fixLeader.LagDist                 = invByteHexToDec(rcvStr.substr(offset*2 + 80, 4));
    fixLeader.CPUSerial               = rcvStr.substr(offset*2 + 84, 16);
    fixLeader.SystemBandwidth         = rcvStr.substr(offset*2 + 100, 5);
    fixLeader.BaseFreqIndex           = invByteHexToDec(rcvStr.substr(offset*2 + 107, 2));
} //end solveFixLeader()

void Decoder::solveVarLeader(ushort offset) {
    //cout << "Decoder::Solving var leader" << endl;

    varLeader.EnsembleNumber          = invByteHexToDec(rcvStr.substr(offset*2 + 4, 4));
    varLeader.TSRecordingTime.Year    = invByteHexToDec(rcvStr.substr(offset*2 + 8, 2));
    varLeader.TSRecordingTime.Month   = invByteHexToDec(rcvStr.substr(offset*2 + 10, 2));
    varLeader.TSRecordingTime.Day     = invByteHexToDec(rcvStr.substr(offset*2 + 12, 2));
    varLeader.TSRecordingTime.Hour    = invByteHexToDec(rcvStr.substr(offset*2 + 14, 2));
    varLeader.TSRecordingTime.Min     = invByteHexToDec(rcvStr.substr(offset*2 + 16, 2));
    varLeader.TSRecordingTime.Sec     = invByteHexToDec(rcvStr.substr(offset*2 + 18, 2));
    varLeader.TSRecordingTime.Sec100  = invByteHexToDec(rcvStr.substr(offset*2 + 20, 2));
    varLeader.EnsembleNumberMSB       = invByteHexToDec(rcvStr.substr(offset*2 + 22, 2));
    varLeader.BITResult               = invByteHexToDec(rcvStr.substr(offset*2 + 24, 4));
    varLeader.SpeedOfSound            = invByteHexToDec(rcvStr.substr(offset*2 + 28, 4));
    varLeader.Depth                   = invByteHexToDec(rcvStr.substr(offset*2 + 32, 4));
    varLeader.Heading                 = invByteHexToDec(rcvStr.substr(offset*2 + 36, 4));
    varLeader.Pitch                   = inv2ndByteHexToDec(rcvStr.substr(offset*2 + 40, 4));
    varLeader.Roll                    = inv2ndByteHexToDec(rcvStr.substr(offset*2 + 44, 4));
    varLeader.Salinity                = invByteHexToDec(rcvStr.substr(offset*2 + 48, 4));
    varLeader.Temperature             = invByteHexToDec(rcvStr.substr(offset*2 + 52, 4));
    varLeader.MPT.Min                 = invByteHexToDec(rcvStr.substr(offset*2 + 56, 2));
    varLeader.MPT.Sec                 = invByteHexToDec(rcvStr.substr(offset*2 + 58, 2));
    varLeader.MPT.Sec100              = invByteHexToDec(rcvStr.substr(offset*2 + 60, 2));
    varLeader.HeadingStddev           = invByteHexToDec(rcvStr.substr(offset*2 + 62, 2));
    varLeader.PitchStddev             = invByteHexToDec(rcvStr.substr(offset*2 + 64, 2));
    varLeader.RollStddev              = invByteHexToDec(rcvStr.substr(offset*2 + 66, 2));
    for (int i=0; i < 8; i++)
    {
        varLeader.ADC[i]              = invByteHexToDec(rcvStr.substr(offset*2 + 68 +2*i, 2));
    }
    for (int i=0; i < 4; i++)
    {
        varLeader.ErrorStt[i]         = invByteHexToDec(rcvStr.substr(offset*2 + 84 + 2*i, 2));
    }
    varLeader.TTRecordingTime.Century = invByteHexToDec(rcvStr.substr(offset*2 + 114, 2));
    varLeader.TTRecordingTime.Year    = invByteHexToDec(rcvStr.substr(offset*2 + 116, 2));
    varLeader.TTRecordingTime.Month   = invByteHexToDec(rcvStr.substr(offset*2 + 118, 2));
    varLeader.TTRecordingTime.Day     = invByteHexToDec(rcvStr.substr(offset*2 + 120, 2));
    varLeader.TTRecordingTime.Hour    = invByteHexToDec(rcvStr.substr(offset*2 + 122, 2));
    varLeader.TTRecordingTime.Min     = invByteHexToDec(rcvStr.substr(offset*2 + 124, 2));
    varLeader.TTRecordingTime.Sec     = invByteHexToDec(rcvStr.substr(offset*2 + 126, 2));
    varLeader.TTRecordingTime.Sec100  = invByteHexToDec(rcvStr.substr(offset*2 + 128, 2));
} //end solveVarLeader()

void Decoder::solveBotTrack(ushort offset) {
    //cout << "Decoder::Solving bottom track" << endl;

    botTrack.PingsPerEnsemble    = invByteHexToDec(rcvStr.substr(offset*2 + 4, 4));
    botTrack.EnsembleWait        = invByteHexToDec(rcvStr.substr(offset*2 + 8, 4));
    botTrack.CorrelationMin      = invByteHexToDec(rcvStr.substr(offset*2 + 12, 2));
    botTrack.AmplitudeMin        = invByteHexToDec(rcvStr.substr(offset*2 + 14, 2));
    botTrack.PctGoodMin          = invByteHexToDec(rcvStr.substr(offset*2 + 16, 2));
    botTrack.BTMode              = invByteHexToDec(rcvStr.substr(offset*2 + 18, 2));
    botTrack.ErrVelocityMax      = invByteHexToDec(rcvStr.substr(offset*2 + 20, 4));
    for (int i=0; i < 4; i++)
    {
        botTrack.Range[i]        = invByteHexToDec(rcvStr.substr(offset*2 + 32 + 4*i, 4));
    }
    for (int i=0; i < 4; i++)
    {
        botTrack.Velocity[i]     = inv2ndByteHexToDec(rcvStr.substr(offset*2 + 48 + 4*i, 4));
    }
    for (int i=0; i < 4; i++)
    {
        botTrack.Correlation[i]  = invByteHexToDec(rcvStr.substr(offset*2 + 64 + 2*i, 2));
    }
    for (int i=0; i < 4; i++)
    {
        botTrack.Amplitude[i]    = invByteHexToDec(rcvStr.substr(offset*2 + 72 + 2*i, 2));
    }
    for (int i=0; i < 4; i++)
    {
        botTrack.PctGood[i]      = invByteHexToDec(rcvStr.substr(offset*2 + 80 + 2*i, 2));
    }
    botTrack.WaterLayerMin       = invByteHexToDec(rcvStr.substr(offset*2 + 88, 4));
    botTrack.WaterLayerNear      = invByteHexToDec(rcvStr.substr(offset*2 + 92, 4));
    botTrack.WaterLayerFar       = invByteHexToDec(rcvStr.substr(offset*2 + 96, 4));
    for (int i=0; i < 4; i++)
    {
        botTrack.WVelocity[i]    = inv2ndByteHexToDec(rcvStr.substr(offset*2 + 100 + 4*i, 4));
    }
    for (int i=0; i < 4; i++)
    {
        botTrack.WCorrelation[i] = invByteHexToDec(rcvStr.substr(offset*2 + 116 + 2*i, 2));
    }
    for (int i=0; i < 4; i++)
    {
        botTrack.WAmplitude[i]   = invByteHexToDec(rcvStr.substr(offset*2 + 124 + 2*i, 2));
    }
    for (int i=0; i < 4; i++)
    {
        botTrack.WPctGood[i]     = invByteHexToDec(rcvStr.substr(offset*2 + 132 + 2*i, 2));
    }
    botTrack.MaxTrackingDepth    = invByteHexToDec(rcvStr.substr(offset*2 + 140, 4));
    for (int i=0; i < 4; i++)
    {
        botTrack.RSSIAmp[i]      = invByteHexToDec(rcvStr.substr(offset*2 + 144 + 2*i, 2));
    }
    botTrack.Gain = invByteHexToDec(rcvStr.substr(offset*2 + 152, 2));
    for (int i=0; i < 4; i++)
    {
        botTrack.RangeMSB[i]     = invByteHexToDec(rcvStr.substr(offset*2 + 154 + 4*i, 4));
    }
} //end solveBotTrack()

bool Decoder::checkRcvEnsemble() {
    if (rcvStr.substr(0,4) != HeaderID) {
        cout << "Decoder::Wrong header ID and data source" << endl;
        return 1;
    }
    int len = rcvStr.length();
    string tobeChecked = rcvStr.substr(0, len - 4);
    string checkStr    = rcvStr.substr(len - 4, 4);
    if(check_checksum(tobeChecked, checkStr)) {
        cout << "Decoder::wrong checksum" << endl;
        cout << "To be checked string: "  << tobeChecked << endl;
        cout << "Checksum: " <<  checkStr << endl;
        return 1;
    }
    return 0;
} //end checkRcvEnsemsble()

bool Decoder::check_checksum(string testStr,string checksum) {
    int len = testStr.length();
    string hexNum;
    int decNum;
    int sum = 0;

    if ((len%2) != 0) {
        cout << "Decoder::Odd string length" << endl;
        return 1;
    }
    for (int i=0; i<len; i+=2) {
        hexNum = testStr.substr(i,2);
        decNum = strtol(hexNum.c_str(),NULL,16);
        /*
        if (decNum == LONG_MAX || decNum == LONG_MIN)
        {
            perror(NULL);
            return 1;
        }*/
        sum += decNum;
    }
    sum = sum % 65536;
    len = checksum.length();
    if (len != 4) return true;
    else decNum = invByteHexToDec(checksum);

    if (decNum == sum) return false;
    
    return true;
} //end check_checksum()

int Decoder::invByteHexToDec(string invHexNum)
{
    int len = invHexNum.length();
    int decNum;
    string hexNum;

    if ((len%2) != 0)
    {
        cout << "Decoder::hexNum length is odd" << endl;
        return -1;
    }
    for (int i=0; i<len; i+=2)
    {
        hexNum = invHexNum.substr(i,2) + hexNum;
    }
    decNum = strtol(hexNum.c_str(),NULL,16);
    /*
    if (decNum == LONG_MAX || decNum == LONG_MIN)
    {
        perror(NULL);
        return 0;
    }*/
    return decNum;
} //end invByteHexToDec()

int Decoder::inv2ndByteHexToDec(string invHexNum)
{
    int binLen = invHexNum.length() * 4;
    int decNum = invByteHexToDec(invHexNum);

    if (decNum >= pow(2, binLen - 1)) decNum = -(pow(2, binLen) - decNum);
    return decNum;
} //end inv2ndByteHexToDec()

void Decoder::printResult()
{
    ostringstream output;

    /*-------print out the header data----------------------------------------*/
    output << "\n HEADER: " << endl;
    output << "Cheksum offset: " << header.ChecksumOffset << endl;
    output << "Number of data types: " << header.NDataTypes << endl;
    for (int i=0; i < header.NDataTypes; i++)
    {
        output << "offset " << i << ": " << header.Offset[i] << endl;
    }

    /*-------print out the fixed leader data----------------------------------*/
    output << "\n FIXED LEADER: " << endl;
    output << "Version: " << fixLeader.CPUFirmware.Version << endl;
    output << "Revision: " << fixLeader.CPUFirmware.Revision << endl;
    output << "Configuration: " << fixLeader.Configuration << endl;
    output << "DummyDataFlag: " << fixLeader.DummyDataFlag << endl;
    output << "Lag: " << fixLeader.Lag << endl;
    output << "Number of beams: " << fixLeader.NBeams << endl;
    output << "Number of Cells: " << fixLeader.NBins << endl;
    output << "Pings per ensemble: " << fixLeader.PingsPerEnsemble << endl;
    output << "Bin Length: " << fixLeader.BinLength << endl;
    output << "Blank after transmit: " << fixLeader.BlankAfterTransmit << endl;
    output << "Signal Process Mode: " << fixLeader.SignalProMode << endl;
    output << "Percentage Correlation Low: " << fixLeader.PctCorrelationLow << endl;
    output << "Number of Code repetition: " << fixLeader.NCodeRepetitions << endl;
    output << "Percentage Good Min: " << fixLeader.PctGoodMin << endl;
    output << "Error Velocity Threshold: " << fixLeader.ErrVelocityThres << endl;
    output << "Time between pings: " << fixLeader.TimeBetweenPings.Min << ": "
        << fixLeader.TimeBetweenPings.Sec << ": " << fixLeader.TimeBetweenPings.Sec100 << endl;
    output << "Coordinate system: " << fixLeader.CoordSystemParms << endl;
    output << "Heading alignment: " << fixLeader.HeadingAlignment << endl;
    output << "Heading bias: " << fixLeader.HeadingBias << endl;
    output << "Sensor source: " << fixLeader.SensorSource << endl;
    output << "Available sensors: " << fixLeader.AvailableSensors << endl;
    output << "Distance to bin 1: " << fixLeader.Bin1Dist << endl;
    output << "Pulse Length: " << fixLeader.PulseLength << endl;
    output << "Start Depth Cell: " << fixLeader.StartDepthCell << endl;
    output << "End Depth Cell: " << fixLeader.EndDepthCell << endl;
    output << "False Target Threshold: " << fixLeader.FalseTargetThres << endl;
    output << "Lag distance: " << fixLeader.LagDist << endl;
    output << "CPU serial: " << fixLeader.CPUSerial << endl;
    output << "System bandwidth: " << fixLeader.SystemBandwidth << endl;
    output << "Base frequency index: " << fixLeader.BaseFreqIndex << endl;

    /*-------print out the variable leader data-------------------------------*/
    output << "\n VARIABLE LEADER DATA TYPE" << endl;
    output << "Ensemble number: " << varLeader.EnsembleNumber << endl;
    output << "Recording time (TS time, without the century): "
        << varLeader.TSRecordingTime.Year << " : "
        << varLeader.TSRecordingTime.Month << " : "
        << varLeader.TSRecordingTime.Day << " : "
        << varLeader.TSRecordingTime.Hour << " : "
        << varLeader.TSRecordingTime.Min << " : "
        << varLeader.TSRecordingTime.Sec << " : "
        << varLeader.TSRecordingTime.Sec100 << " : " << endl;
    output << "Ensemble number MSB: " << varLeader.EnsembleNumberMSB << endl;
    output << "Built in test result: " << varLeader.BITResult << endl;
    output << "Speed of sound: " << varLeader.SpeedOfSound << endl;
    output << "Depth: " << varLeader.Depth << endl;
    output << "Heading: " << varLeader.Heading << endl;
    output << "Pitch: " << varLeader.Pitch << endl;
    output << "Roll: " << varLeader.Roll << endl;
    output << "Salinity: " << varLeader.Salinity << endl;
    output << "Temperature: " << varLeader.Salinity << endl;
    output << "Minimum pre-ping wait time: " << varLeader.MPT.Min << " : "
        << varLeader.MPT.Sec << " : "
        << varLeader.MPT.Sec100 << " : " << endl;
    output << "Heading standard deviation: " << varLeader.HeadingStddev << endl;
    output << "Pitch standard deviation: " << varLeader.PitchStddev << endl;
    output << "Roll standard deviation: " << varLeader.RollStddev << endl;

    output << "Analog digital converter 1 to 8: ";
    for (int i=0; i < 8; i++)
    {
        output << varLeader.ADC[i] << ": ";
    }
    output << endl;

    output << "Error status: ";
    for (int i=0; i < 4; i++)
    {
        output << varLeader.ErrorStt[i] << ": ";
    }
    output << endl;

    output << "Pressure: " << varLeader.Pressure << endl;
    output << "Pressure variance: " << varLeader.PressureVar << endl;

    output << "Recording time (TT time, with the century): "
        << varLeader.TTRecordingTime.Century << " : "
        << varLeader.TTRecordingTime.Year << " : "
        << varLeader.TTRecordingTime.Month << " : "
        << varLeader.TTRecordingTime.Day << " : "
        << varLeader.TTRecordingTime.Hour << " : "
        << varLeader.TTRecordingTime.Min << " : "
        << varLeader.TTRecordingTime.Sec << " : "
        << varLeader.TTRecordingTime.Sec100 << " : " << endl;

    /*-------print out the bottom tracking data-------------------------------*/
    output << "\n BTTOM TRACK DATA" << endl;
    output << "Pings per ensemble: " << botTrack.PingsPerEnsemble << endl;
    output << "Ensemble wait: " << botTrack.EnsembleWait << endl;
    output << "Correlation Min: " << botTrack.CorrelationMin << endl;
    output << "Amplitude Min: " << botTrack.AmplitudeMin << endl;
    output << "Percentage good min: " << botTrack.PctGoodMin << endl;
    output << "BT mode: " << botTrack.BTMode << endl;
    output << "Error velocity max: " << botTrack.ErrVelocityMax << endl;

    output << "Range: ";
    for (int i=0; i < 4; i++)
    {
        output << botTrack.Range[i] << ": ";
    }
    output << endl;

    output << "Velocity: ";
    for (int i=0; i < 4; i++)
    {
        output <<botTrack.Velocity[i] << ": ";
    }
    output << endl;

    output << "Correlation: ";
    for (int i=0; i < 4; i++)
    {
        output << botTrack.Correlation[i] << ": ";
    }
    output << endl;

    output << "Amplitude: ";
    for (int i=0; i < 4; i++)
    {
        output << botTrack.Amplitude[i] << ": ";
    }
    output << endl;

    output << "Percentage good: ";
    for (int i=0; i < 4; i++)
    {
        output << botTrack.PctGood[i] << ": ";
    }
    output << endl;

    output << "Water layer min, near and far: " << botTrack.WaterLayerMin << ": "
        << botTrack.WaterLayerNear << ": "
        << botTrack.WaterLayerFar << endl;
    output << "Water reference velocity: ";
    for (int i=0; i < 4; i++)
    {
        output << botTrack.WVelocity[i] << ": ";
    }
    output << endl;

    output << "Water reference correlation: ";
    for (int i=0; i < 4; i++)
    {
        output << botTrack.WCorrelation[i] << ": ";
    }
    output << endl;

    output << "Water reference amplitude: ";
    for (int i=0; i < 4; i++)
    {
        output << botTrack.WAmplitude[i] << ": ";
    }
    output << endl;

    output << "Water reference percentage good: ";
    for (int i=0; i < 4; i++)
    {
        output <<botTrack.WPctGood[i] << ": ";
    }
    output << endl;

    output << "Max tracking depth: " << botTrack.MaxTrackingDepth << endl;

    output << "Receiver signal streng indicator amplitude: ";
    for (int i=0; i < 4; i++)
    {
        output << botTrack.RSSIAmp[i] << ": ";
    }
    output << endl;

    output << "Gain: " << botTrack.Gain << endl;

    output << "Range MSB: ";
    for (int i=0; i < 4; i++)
    {
        output << botTrack.RangeMSB[i] << ": ";
    }
    output << endl;

    printStr = output.str();
    cout << "Decoder::getting print string" << endl;
    return;
} //end printResult()

/*-----------------setters and getters----------------------*/
void Decoder::setRcvStr(string str)
{
    rcvStr = str;
    return;
}

string Decoder::getPrintStr()
{
    printResult();
    return printStr;
}

Decoder::HeaderType Decoder::getHeader()
{
    return header;
}

Decoder::FixLeaderType Decoder::getFixLeader()
{
    return fixLeader;
}

Decoder::VarLeaderType Decoder::getVarLeader()
{
    return varLeader;
}

Decoder::BottomTrackType Decoder::getBotTrack()
{
    return botTrack;
}
