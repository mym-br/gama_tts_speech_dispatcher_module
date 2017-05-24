/***************************************************************************
 *  Copyright 2015, 2017 Marcelo Y. Matuda                                 *
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

#include "ModuleConfiguration.h"

#include "Util.h"

namespace {

const std::string  someStr{"some"};
const std::string   allStr{"all"};
const std::string    onStr{"on"};
const std::string spellStr{"spell"};
const std::string  iconStr{"icon"};
const std::string  nullStr{"NULL"};

}

ModuleConfiguration::ModuleConfiguration()
		: logLevel{}
		, pitch{}
		, rate{}
		, volume{}
		, punctuationMode(PUNCTUATION_NONE)
		, spellingMode{}
		, capitalLetterRecognition(CAPITAL_LETTER_RECOGNITION_NONE)
{
}

void
ModuleConfiguration::setLogLevel(const std::string& s)
{
	unsigned int value = Util::convertString<unsigned int>(s);
	if (value > 5) value = 5;
	logLevel = value;
}

void
ModuleConfiguration::setPitch(const std::string& s)
{
	const float value = Util::convertString<float>(s);
	pitch = Util::bound(-100.0f, value, 100.0f);
}

void
ModuleConfiguration::setRate(const std::string& s)
{
	const float value = Util::convertString<float>(s);
	rate = Util::bound(-100.0f, value, 100.0f);
}

void
ModuleConfiguration::setVolume(const std::string& s)
{
	const float value = Util::convertString<float>(s);
	volume = Util::bound(-100.0f, value, 100.0f);
}

void
ModuleConfiguration::setPunctuationMode(const std::string& s)
{
	if (s == someStr) {
		punctuationMode = PUNCTUATION_SOME;
	} else if (s == allStr) {
		punctuationMode = PUNCTUATION_ALL;
	} else { // "none"
		punctuationMode = PUNCTUATION_NONE;
	}
}

void
ModuleConfiguration::setSpellingMode(const std::string& s)
{
	if (s == onStr) {
		spellingMode = true;
	} else { // "off"
		spellingMode = false;
	}
}

void
ModuleConfiguration::setCapitalLetterRecognition(const std::string& s)
{
	if (s == spellStr) {
		capitalLetterRecognition = CAPITAL_LETTER_RECOGNITION_SPELL;
	} else if (s == iconStr) {
		capitalLetterRecognition = CAPITAL_LETTER_RECOGNITION_ICON;
	} else { // "none"
		capitalLetterRecognition = CAPITAL_LETTER_RECOGNITION_NONE;
	}
}

void
ModuleConfiguration::setVoice(const std::string& s)
{
	if (s == nullStr) {
		voice = std::string{};
	} else {
		voice = s;
	}
}

void
ModuleConfiguration::setLanguage(const std::string& s)
{
	if (s == nullStr) {
		language = std::string{};
	} else {
		language = s;
	}
}

void
ModuleConfiguration::setSynthesisVoice(const std::string& s)
{
	if (s == nullStr) {
		synthesisVoice = std::string{};
	} else {
		synthesisVoice = s;
	}
}
