// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_DATASTRUCTURES_PLYREADER_H
#define SURGSIM_DATASTRUCTURES_PLYREADER_H

#include <string>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

namespace SurgSim
{
namespace DataStructures
{

class PlyReaderDelegate;

/// Wrapper for the C .ply file parser
/// This class wraps the main functionality for the original C .ply file parser at
/// http://paulbourke.net/dataformats/ply/
/// it enables customization of the parsing process either through a delegate class or through executing
/// the requestElement and requestProperty functions.
/// The file needs to be a valid .ply file, either ascii or binary, for the reader to be a valid reader.
/// ## General Information
/// A ply file consists of a header description followed by the data, a header might look like this
///   ply
///   	format ascii 1.0           { ascii/binary, format version number }
///   comment made by Greg Turk  { comments keyword specified, like all lines }
///   comment this file is a cube
///   	element vertex 8           { define "vertex" element, 8 of them in file }
///   property float x           { vertex contains float "x" coordinate }
///   property float y           { y coordinate is also a vertex property }
///   property float z           { z coordinate, too }
///   element face 6             { there are 6 "face" elements in the file }
///   property list uchar int vertex_indices { "vertex_indices" is a list of ints }
///   end_header                 { delimits the end of the header }
///
/// As you can see there are elements with properties, there can be multiple elements and multiple
/// properties per element. An element can have scalar and/or list properties.
/// To work correctly users will have to preallocate the memory that will be used by the parser
/// to deposit the information from the file, and set offsets to the correct value to match
/// the expected locations in the preallocated receiving memory.
///
/// ## Initialisation
/// The constructor for the PlyReader accepts the file name of the file, it will make sure
/// the the file exists and is actually a .ply file, after the constructor has executed
/// isValid() should be true. At this time all the information in the header is available and
/// users of the reader can tell the reader which elements and which properties are of interest
/// using the requestElement() and requestProperty() functions.
///
/// ## The three types of callback functions
/// Parsing is accomplished via a set of callback functions. There are three kinds of callback
/// function each with a different responsibility.
/// - The BeginElement callback is called whenever the section for an element is encountered in
/// 	the file, it will pass the name of the element and the number of elements that will be read
/// 	it should return a pointer to the preallocated memory that will be used for reading.
///
/// - The ProcessElement callback is called whenever a new element has been read from the file,
/// 	the read values can be copied from the preallocated memory that was sent to the parser in the
/// 	previous callback. If list data was requested, the memory for the list data needs to be freed
/// 	at this time.
///
/// - The EndElement callback is called whenever processing of the element is concluded, this
/// 	gives users a chance to finalize processing on their side.
///
/// ## The delegate
/// The PlyReaderDelegate interface should be used to create classes that encapsulate the needed
/// callbacks and their processing.


class PlyReader
{
public:
	/// Values that represent the data type/size of requested data.
	enum Type
	{
		TYPE_INVALID = 0,
		TYPE_CHAR,
		TYPE_SHORT,
		TYPE_INT,
		TYPE_UNSIGNED_CHAR,
		TYPE_UNSIGNED_SHORT,
		TYPE_UNSIGNED_INT,
		TYPE_FLOAT,
		TYPE_DOUBLE,
		TYPE_COUNT
	};

	/// The callback that is being used to indicate the start of an element, the parameters that are passed
	/// into the callback are the name of the element that is being processed and the number of elements
	/// that will be processed. The callback needs to return allocated memory that is used by the reader
	/// to deposit the read information
	typedef std::function<void* (const std::string&, size_t)> StartElementCallbackType;

	/// The callback that is used for the processing and the end of processing, the parameter passed is
	/// the name of the element that is being processed.
	typedef std::function<void (const std::string&)> StandardCallbackType;

	/// Constructor.
	/// \param	filename	Filename of the .ply file.
	explicit PlyReader(const std::string& filename);

	/// Destructor.
	virtual ~PlyReader();

	/// Query if this object is valid.
	/// \return	true if the file was successfully opened, false if not.
	bool isValid() const;

	/// Request element to be processed during parsing.
	///
	/// \param	elementName Name of the element that is needed.
	/// \param	startElementCallback The callback to be used when the element is first encountered.
	/// \param	processElementCallback	The callback to be used when one element of this type is read.
	/// \param	endElementCallback	  	The callback to be used when all of the elements of this type have been read.
	///
	/// \return	true if there is a element elementName in the .ply file and it has not been requested yet.
	bool requestElement(const std::string& elementName,
						std::function<void* (const std::string&, size_t)> startElementCallback,
						std::function<void (const std::string&)> processElementCallback,
						std::function<void (const std::string&)> endElementCallback);

	/// Request a scalar property for parsing.
	/// Use this for when you want the information from a scalar property from the .ply file. With this call
	/// you register the type that you want for storing the data and the offset in the data structure where the
	/// information should be stored. The data actually comes from the startElementCallback that was supplied
	/// in the previous call
	/// \warning If the offset is wrong or the data type provided and the actual data type in your structure
	///          does not match there could be a buffer overrun, use this with caution
	/// \param elementName  Name of the element that contains this property.
	/// \param propertyName Name of the property that you want stored.
	/// \param dataType	The type of the data that should be stored.
	/// \param dataOffset The offset of the data in your data structure where the data should be stored.
	/// \return true if the property exists and has not been registered yet and is a scalar property,
	/// 		otherwise false.
	bool requestScalarProperty(const std::string& elementName, const std::string& propertyName,
		int dataType, int dataOffset);

	/// Request a list property for parsing.
	/// Use this for when you want the information from a list property from the .ply file. The item in your
	/// data structure should be a pointer of the type of data that you want, the reader will allocate the needed
	/// space and deposit all the items in the list in this space.
	/// \warning If the offset is wrong or the data type provided and the actual data type in your structure
	/// 	  does not match there could be a buffer overrun, use this with caution.
	/// \param elementName  Name of the element that contains this property.
	/// \param propertyName Name of the property that you want stored.
	/// \param dataType	The type of the data that should be stored.
	/// \param dataOffset The offset of the data in your data structure where the data should be stored.
	/// \param countType The type of the number of element that should be stored.
	/// \param countOffset The offset for storing the count.
	///
	/// \return true if it succeeds, false if it fails.
	bool requestListProperty(const std::string& elementName,
							 const std::string& propertyName,
							 int dataType, int dataOffset,
							 int countType, int countOffset);

	/// Query if this elementName is in the .ply file
	/// \param elementName Name of the element.
	/// \return true if yes, false otherwise.
	bool hasElement(const std::string& elementName) const;

	/// Query if 'elementName' has the given property.
	/// \param elementName  Name of the element.
	/// \param propertyName Name of the property.
	/// \return true if the element exists and has the property, false otherwise.
	bool hasProperty(const std::string& elementName, const std::string& propertyName) const;

	/// Query if the property of the give element is scalar.
	/// \param elementName  Name of the element.
	/// \param propertyName Name of the property.
	/// \return true if the element exists and has the property and it is a scalar value.
	bool isScalar(const std::string& elementName, const std::string& propertyName) const;

	/// Sets a delegate for parsing and then parse the file.
	/// \param delegate The delegate.
	/// \return true if set and parse are successful; otherwise false.
	bool parseWithDelegate(std::shared_ptr<PlyReaderDelegate> delegate);

	/// Register callback to be called at the begining of parseFile.
	void setStartParseFileCallback(std::function<void (void)> startParseFileCallback);

	/// Register callback to be called at the end of parseFile.
	void setEndParseFileCallback(std::function<void (void)> endParseFileCallback);

	/// Sets a delegate for parsing.
	/// \param delegate The delegate.
	/// \return true if it succeeds and the properties in the ply file satisfy the delegates fileIsAcceptable().
	bool setDelegate(std::shared_ptr<PlyReaderDelegate> delegate);

	/// Parse the file.
	void parseFile();

private:
	friend class PlyReaderTests;

	/// Generic Internal function to handle list and scalar properties, see requestScalarProperty() and
	/// requestListProperty() for full documentation.
	/// \param elementName  Name of the element that contains this property.
	/// \param propertyName Name of the property that you want stored.
	/// \param dataType	The type of the data that should be stored.
	/// \param dataOffset The offset of the data in your data structure where the data should be stored.
	/// \param countType The type of the number of element that should be stored.
	/// \param countOffset The offset for storing the count.
	///
	/// \return true if it succeeds, false if it fails.
	bool requestProperty(const std::string& elementName,
						 const std::string& propertyName,
						 int dataType, int dataOffset,
						 int countType, int countOffset);

	/// The name of the .ply file
	std::string m_filename;

	/// Information about the property on the .ply file.
	struct PropertyInfo
	{
		std::string propertyName; ///< Name of the property.
		int dataType = -1; ///< Type of the receiving data.
		int dataOffset = -1; ///< Location for the receiving data.
		int countType = -1; ///< For lists, type of the receiving data for the count of listelements.
		int countOffset = -1; ///< For lists, location of the receiving data for the count.
	};

	/// Information about the element in the .ply file.
	struct ElementInfo
	{
		std::string name; ///< Name of the element
		StartElementCallbackType startElementCallback; ///< Callback to be used when the element is first encountered.
		StandardCallbackType processElementCallback; ///< Callback to be used for each processed element.
		StandardCallbackType endElementCallback; ///< Callback to be used after all the elements have been processed.
		std::vector<PropertyInfo> requestedProperties; ///< All the properties that are wanted
	};

	std::unordered_map<std::string, ElementInfo> m_requestedElements;

	///@{
	/// Pimpl Data to wrap ply reader local information
	struct Data;
	std::unique_ptr<Data> m_data;
	///@}

	/// The delegate.
	std::shared_ptr<PlyReaderDelegate> m_delegate;

	/// Callback to be executed at the start of 'parseFile'.
	std::function<void(void)> m_startParseFileCallback;

	/// Callback to be executed at the end of 'parseFile'.
	std::function<void(void)> m_endParseFileCallback;
};

} // DataStructures
} // SurgSim

#endif // SURGSIM_DATASTRUCTURES_PLYREADER_H