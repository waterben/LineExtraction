#include <utility/string_table.hpp>

#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <fstream>

namespace lsfm {

    std::vector<std::string> StringTable::col(size_t c) const {
        std::vector<std::string> ret;
        size_t s = size();
        for (size_t p = c; p < s; p += cols_)
            ret.push_back(data_[p]);
        return ret;
    }

    StringTable StringTable::transpose() const {
        StringTable ret(cols_, rows_);
        for (size_t r = 0; r < rows_; ++r)
            for (size_t c = 0; c < cols_; ++c)
                ret(c, r) = this->operator()(r, c);
        return ret;
    }

    void StringTable::saveCSV(const std::string& file) const {
        std::ofstream ofs;
        ofs.open(file.c_str());

        size_t col = 0;
        for_each(data_.begin(), data_.end(), [&](const std::string &cell) {
            ++col;
            ofs << cell << ";";
            if (col % cols_ == 0)
                ofs << std::endl;
        });
        ofs.close();
    }

    std::ostream& operator<<(std::ostream& os, const StringTable& StringTable) {
        size_t col = 0;
        for_each(StringTable.data().begin(), StringTable.data().end(), [&](std::string cell) {
            ++col;
            if (col < 8) {
                if (cell.size() > 15)
                    cell = cell.substr(0, 15);
                std::cout << cell << (cell.size() < 8 ? "\t\t" : "\t");
            }
            if (col == StringTable.cols()) {
                col = 0;
                std::cout << "#" << std::endl;
            }
        });
        //std::cout << std::endl;
        return os;
    }
}

