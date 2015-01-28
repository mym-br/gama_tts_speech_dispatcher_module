/***************************************************************************
 *  Copyright 2015 Marcelo Y. Matuda                                       *
 *                                                                         *
 *  This program is free software: you can redistribute it and/or modify   *
 *  it under the terms of the GNU General Public License as published by   *
 *  the Free Software Foundation, either version 3 of the License, or      *
 *  (at your option) any later version.                                    *
 *                                                                         *
 *  This program is distributed in the hope that it will be useful,        *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *  GNU General Public License for more details.                           *
 *                                                                         *
 *  You should have received a copy of the GNU General Public License      *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.  *
 ***************************************************************************/

#ifndef UTIL_H_
#define UTIL_H_

#include <sstream>
#include <string>
#include <utility> /* make_pair, pair */



namespace Util {

void ignoreTags(std::string& msg);
std::pair<std::string, std::string> getNameAndValue(const std::string& s);
void removeLF(std::string& msg);
template<typename T> T convertString(const std::string& s);



template<typename T>
T
convertString(const std::string& s) {
	std::istringstream in(s);
	T res;
	in >> res;
	if (!in) return 0;
	return res;
}

} // namespace Util

#endif /* UTIL_H_ */
