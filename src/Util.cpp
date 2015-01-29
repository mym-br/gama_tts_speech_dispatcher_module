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

#include "Util.h"



namespace Util {

void
stripSSML(std::string& msg)
{
	std::string dest(msg.size(), ' ');

	std::string::size_type pos = 0;
	std::string::size_type destPos = 0;
	const std::string::size_type size = msg.size();
	while (pos < size) {
		if (msg[pos] == '<') {
			dest[destPos++] = ' ';
			std::string::size_type findPos = msg.find('>', pos + 1);
			if (findPos == std::string::npos) {
				pos = size;
			} else {
				pos = findPos + 1;
			}
			continue;
		} else if (msg[pos] == '&') {
			const std::string::size_type nextPos = pos + 1;
			if (msg.find("lt;", nextPos) == nextPos) {
				dest[destPos++] = '<';
				pos += 4;
				continue;
			}
			if (msg.find("gt;", nextPos) == nextPos) {
				dest[destPos++] = '>';
				pos += 4;
				continue;
			}
			if (msg.find("amp;", nextPos) == nextPos) {
				dest[destPos++] = '&';
				pos += 5;
				continue;
			}
		}
		dest[destPos++] = msg[pos++];
	}

	dest.resize(destPos);
	msg.swap(dest);
}

std::pair<std::string, std::string>
getNameAndValue(const std::string& s)
{
	std::string::size_type pos = s.find('=');
	if (pos == std::string::npos || pos == s.size() - 1) {
		return std::make_pair(s, std::string());
	}
	return std::make_pair(s.substr(0, pos), s.substr(pos + 1, s.size() - pos - 1));
}

void
removeLF(std::string& msg)
{
	for (char& c : msg) {
		if (c == '\n') {
			c = ' ';
		}
	}
}

} // namespace Util
