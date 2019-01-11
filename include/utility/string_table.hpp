#ifndef _STRING_TABLE_HPP_
#define _STRING_TABLE_HPP_
#ifdef __cplusplus

#include <iostream>
#include <string>
#include <vector>

namespace lsfm {

    class StringTable {
        size_t rows_, cols_;
        std::vector<std::string> data_;

    public:
        StringTable(size_t rows = 1, size_t cols = 1) : rows_(rows), cols_(cols) {
            data_.resize(rows_ * cols_);
        }

        size_t rows() const {
            return rows_;
        }

        size_t cols() const {
            return cols_;
        }

        size_t size() const {
            return rows_ * cols_;
        }

        std::string& operator()(size_t row, size_t col) {
            return data_[row * cols_ + col];
        }

        const std::string& operator()(size_t row, size_t col) const {
            return data_[row * cols_ + col];
        }

        std::string* operator[](size_t row) {
            return &data_[row * cols_];
        }

        const std::string* operator[](size_t row) const {
            return &data_[row * cols_];
        }

        std::vector<std::string>& data() {
            return data_;
        }

        const std::vector<std::string>& data() const {
            return data_;
        }

        std::vector<std::string> row(size_t r) const {
            return std::vector<std::string>(data_.begin() + r * cols_, data_.begin() + r + 1 * cols_);
        }

        std::vector<std::string> col(size_t c) const;

        StringTable transpose() const;

        void saveCSV(const std::string& file) const;
    };

    std::ostream& operator<<(std::ostream& os, const StringTable& StringTable);

}

#endif
#endif