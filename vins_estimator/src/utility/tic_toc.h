/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc {
  public:
	TicToc() {
		tic();
	}

	void tic() {
		start = std::chrono::system_clock::now();
	}

	double toc() {
		end											  = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end - start;
		elapsed_sum += elapsed_seconds.count();
		count++;
		cur_ms = elapsed_seconds.count() * 1000;
		avg_ms = elapsed_sum / count * 1000;
		return elapsed_seconds.count() * 1000;
	}
	void reset() {
		cur_ms		= 0;
		avg_ms		= 0;
		elapsed_sum = 0;
		count		= 0;
	};
	double cur_ms, avg_ms;

  private:
	std::chrono::time_point<std::chrono::system_clock> start, end;
	double											   elapsed_sum = 0;
	int												   count	   = 0;
};


// ANSI颜色码
#define ANSI_COLOR_RESET   "\033[0m"  /*color reset */
#define ANSI_COLOR_RED	   "\033[31m" /* Red */
#define ANSI_COLOR_GREEN   "\033[32m" /* Green */
#define ANSI_COLOR_YELLOW  "\033[33m" /* Yellow */
#define ANSI_COLOR_BLUE	   "\033[34m" /* Blue */
#define ANSI_COLOR_MAGENTA "\033[35m" /* Magenta */
#define ANSI_COLOR_CYAN	   "\033[36m" /* Cyan */
#define ANSI_COLOR_WHITE   "\033[37m" /* White */