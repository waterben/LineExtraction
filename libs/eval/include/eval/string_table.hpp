#pragma once

#include <string>
#include <vector>
#include <ostream>

namespace lsfm {

/// @brief A 2D table of strings with CSV export capabilities
class StringTable {
public:
    /// @brief Default constructor
    StringTable() : rows_(0), cols_(0) {}
    
    /// @brief Constructor with dimensions
    /// @param rows Number of rows
    /// @param cols Number of columns
    StringTable(size_t rows, size_t cols) : rows_(rows), cols_(cols), data_(rows * cols) {}
    
    /// @brief Get/set element at position (row, col)
    /// @param row Row index
    /// @param col Column index
    /// @return Reference to string at position
    std::string& operator()(size_t row, size_t col) {
        return data_[row * cols_ + col];
    }
    
    /// @brief Get element at position (row, col) (const version)
    /// @param row Row index
    /// @param col Column index
    /// @return Const reference to string at position
    const std::string& operator()(size_t row, size_t col) const {
        return data_[row * cols_ + col];
    }
    
    /// @brief Get all elements in a column
    /// @param c Column index
    /// @return Vector of strings in column c
    std::vector<std::string> col(size_t c) const;
    
    /// @brief Transpose the table
    /// @return Transposed StringTable
    StringTable transpose() const;
    
    /// @brief Save table as CSV file
    /// @param file Output filename
    void saveCSV(const std::string& file) const;
    
    /// @brief Get number of rows
    /// @return Number of rows
    size_t rows() const { return rows_; }
    
    /// @brief Get number of columns
    /// @return Number of columns
    size_t cols() const { return cols_; }
    
    /// @brief Get total number of elements
    /// @return Total number of elements
    size_t size() const { return data_.size(); }
    
    /// @brief Get raw data vector
    /// @return Const reference to data vector
    const std::vector<std::string>& data() const { return data_; }

private:
    size_t rows_;
    size_t cols_;
    std::vector<std::string> data_;
};

/// @brief Stream output operator for StringTable
/// @param os Output stream
/// @param table StringTable to output
/// @return Reference to output stream
std::ostream& operator<<(std::ostream& os, const StringTable& table);

}  // namespace lsfm