#ifndef _CMDCLI_H
#define _CMDCLI_H

#include <CmdCallback.hpp>
#include <CmdParser.hpp>
#include "OwDevices.h"

class CmdCli
{
	private:
		CmdCallback<14> cmdCallback;
		CmdParser cmdParser;
		/**
		 * /param prefix check for existing prefix '0x' for hex or dec
		 * */
		uint8_t atoh(const char *str, bool prefix = true);
		void dumpSwTbl(void);

		static void funcBus(CmdParser *myParser);
		static void funcPinSet(CmdParser *myParser);
		static void funcPinGet(CmdParser *myParser);
		static void funcMode(CmdParser *myParser);
		static void funcData(CmdParser *myParser);
		static void funcPio(CmdParser *myParser);
		static void funcStatus(CmdParser *myParser);
		static void funcSearch(CmdParser *myParser);
		static void funcCfg(CmdParser *myParser);
		static void funcCmd(CmdParser *myParser); /* 10 */
		static void funcSwCmd(CmdParser *myParser);
		static void funcTemp(CmdParser *myParser); /* 12 */
		static void funcTime(CmdParser *myParser);
		static void funcLog(CmdParser *myParser);

public:
	CmdCli() {;}
	void begin(OwDevices* devs);
	void end();
	void loop();
	static void resetInput();
};

#endif