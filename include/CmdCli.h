#ifndef _CMDCLI_H
#define _CMDCLI_H

#include <CmdCallback.hpp>
#include <CmdParser.hpp>
#include "OwDevices.h"

class CmdCli
{
	private:
		CmdCallback<10> cmdCallback;
		CmdParser cmdParser;

		static void funcBus(CmdParser *myParser);
		static void funcAlarmSrch(CmdParser *myParser);
		static void funcPinSet(CmdParser *myParser);
		static void funcMode(CmdParser *myParser);
		static void funcTest(CmdParser *myParser);
		static void funcData(CmdParser *myParser);
		static void funcPio(CmdParser *myParser);
		static void funcStatus(CmdParser *myParser);
		static void funcSearch(CmdParser *myParser);
		static void funcCfg(CmdParser *myParser);
		static void funcCmd(CmdParser *myParser);

public:
	CmdCli() {;}
	void begin(OwDevices* devs);
	void end();
	void loop();
	static void resetInput();
};

#endif