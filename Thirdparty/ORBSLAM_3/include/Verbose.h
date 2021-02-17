namespace ORB_SLAM3
{
    class Verbose
    {
    public:
        enum eLevel
        {
            VERBOSITY_QUIET=0,
            VERBOSITY_NORMAL=1,
            VERBOSITY_VERBOSE=2,
            VERBOSITY_VERY_VERBOSE=3,
            VERBOSITY_DEBUG=4
        };

        static eLevel th;

    public:
        static void PrintMess(std::string str, eLevel lev)
        {
            if(lev <= th)
                cout << str << endl;
        }

        static void SetTh(eLevel _th)
        {
            th = _th;
        }
    };
}