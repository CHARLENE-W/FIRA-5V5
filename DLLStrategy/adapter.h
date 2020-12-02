#pragma once

#include <string>
/* If you want to use fmt, uncomment me
#include <fmt/format.h>
*/

namespace Adapter {

	/*
	Sends a message to V5DLLAdapter.
	lpMessage: The message text, in UTF-16LE encoding.
	dwSizeInBytes: The message length in bytes. Does not include the null character.
	*/
	bool SendLog(LPCWSTR lpMessage, DWORD dwSizeInBytes) {
		static struct {
			HWND hWnd = nullptr;
			DWORD dwMyPid = GetCurrentProcessId();
		} params;
		static const DWORD ADAPTER_MAGIC = 0x56352B2B;
		static const DWORD DATA_LOG = 1;
		if (params.hWnd == nullptr) {
			auto enumProc = [](HWND hWnd, LPARAM lParam)->BOOL {
				auto& p = *(decltype(params)*)lParam;
				DWORD dwTargetPid = 0;
				GetWindowThreadProcessId(hWnd, &dwTargetPid);
				if (dwTargetPid == p.dwMyPid) {
					LONG magic = GetWindowLongW(hWnd, GWLP_USERDATA);
					if (magic == ADAPTER_MAGIC) {
						p.hWnd = hWnd;
						return FALSE;
					}
				}
				return TRUE;
			};
			EnumWindows((WNDENUMPROC)enumProc, (LPARAM)&params);
		}
		if (params.hWnd != nullptr) {
			COPYDATASTRUCT cds;
			cds.dwData = DATA_LOG;
			cds.cbData = dwSizeInBytes;
			cds.lpData = (LPVOID)lpMessage;
			return SendMessageW(params.hWnd, WM_COPYDATA, NULL, (LPARAM)&cds);
		}
		return false;
	}

	//You are encouraged to create your own wrapper for this.

	/*
	Sends a message to V5DLLAdapter.
	message: The message text.
	*/
	bool SendLog(const std::wstring& message) {
		return SendLog(message.c_str(), message.size() * sizeof(wchar_t));
	}

	/* If you want to use fmt, uncomment me
	template<typename ...Args>
	bool Log(fmt::WCStringRef format, Args&& ...args) {
		auto str = fmt::format(format, std::forward<Args>(args)...);
		return SendLog(str.c_str(), str.size() * sizeof(decltype(str)::value_type));
	}
	*/
}
