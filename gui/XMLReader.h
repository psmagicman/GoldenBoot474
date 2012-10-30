#ifndef __XMLREADER__
#define __XMLREADER__

#include <string>
#include <vector>
#include <limits>
#include "tinyxml2.h"

//
// This Data structure stores information about a value from the XML
// tag: Stores the tag (i.e. <tag></tag>)
// name: Stores a name that describes the tag
// value: Stores the value (i.e. <tag>value</tag>)
// max: Stores the maximum limit for the value. *MAKE SURE value IS A NUMBER
// min: Stores the minimum
// limit: Signifies if this data requires a maximum/minimum limit
// type: Used for GUI to determine what type of edit to use: lineEdit, checkBox, spinBox etc.
// extra: Stores extra values for miscellaneous tasks (i.e. dropBox)
// extraValue: Stores extra values for miscellaneous tasks (i.e.dropBox)
//
struct Data{
	std::string tag;
	std::string value;
};

class XMLReader
{
public:
	XMLReader();
	XMLReader(std::string path);
	// Saves the XML to specified path. If path is empty, saves to where the XML is opened
	bool save(std::string path = "");

	// Returns the number of tags in the XML. Example:
	// <tag1>value1</tag1>
	// <tag2>value2</tag2>
	// getSize() returns '2'
	int getSize();

	// Returns the full path of the XML (i.e. C:\path\file.xml)
	std::string getPath();
	// Returns the index of the tag. The index is the position of the tag in the xml. Example XML:
	// <tag1>value1</tag1>
	// <tag2>value2</tag2>
	// getIndex(tag1) returns '0'. while getIndex(tag2) returns '1'
	int getIndex(std::string tag);
	std::string getTag(int index);
	std::string getValue(int index);
	std::string operator [] (int index);
	std::string getValue(std::string tag);
	std::string operator [] (std::string tag);

	// Overwrite a current tag with a new tag
	bool setTag(std::string tag, std::string newTag);
	// Overwrite a value with a new value with index as parameter
	void setValue(int iData, std::string value);
	// Overwrite a value with a new value with tag as parameter
	void setValue(std::string tag, std::string value);
	// Set the minimum and maximum limits

	// Insert a new Data struct into the existing XML file
	void insertData(std::string tag, std::string value);
	// Remove a Data with tag
	bool removeData(std::string tag);

private:
	int _size;
	std::string _path;
	std::vector<Data> _data;
};

#endif
