#ifndef _CMDCLI_H
#define _CMDCLI_H

#include <CmdCallback.hpp>
#include <CmdParser.hpp>
#include "OwDevices.h"

#define CLI_NR_BASE 9
#ifdef EXT_DEBUG
#define CLI_NR CLI_NR_BASE + 3
#else
#define CLI_NR CLI_NR_BASE
#endif

class CmdCli
{
	private:
		CmdCallback<CLI_NR> cmdCallback;
		CmdParser cmdParser;
		/**
		 * /param prefix check for existing prefix '0x' for hex or dec
		 * */
		uint8_t atoh(const char *str, bool prefix = true);
		void dumpSwTbl(void);

		static void funcBus(CmdParser *myParser);
		static void funcMode(CmdParser *myParser);
		static void funcPio(CmdParser *myParser);
		static void funcStatus(CmdParser *myParser);
		static void funcSearch(CmdParser *myParser); /* 5 */
		static void funcCfg(CmdParser *myParser);
		static void funcCmd(CmdParser *myParser); /* 7 */
		static void funcSwCmd(CmdParser *myParser);
		static void funcTemp(CmdParser *myParser); /* 9 */
		//static void funcTime(CmdParser *myParser);
#ifdef EXT_DEBUG
		static void funcPinSet(CmdParser *myParser);
		static void funcPinGet(CmdParser *myParser);
		static void funcLog(CmdParser *myParser);
#endif

public:
	CmdCli() {;}
	void begin(OwDevices* devs);
	void end();
	void loop();
	static void resetInput();
};

#endif