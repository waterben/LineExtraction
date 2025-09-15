#include <sstream>
#include <qplot/PlotManager.h>

using namespace std;

std::string PlotManager::new_name() {
    ++id_count_;
    ostringstream os;
    os << "figure_" << id_count_;
    return os.str();
}

// get list of figure names
std::vector<std::string> PlotManager::nameList() const {
    std::vector<std::string> ret;
    for_each(fmap_.begin(), fmap_.end(), [&](const std::pair<std::string, PlotWindow*>& data) {
        ret.push_back(data.first);
    });
    return ret;
}

//! get figure pointer (create new if not exist)
PlotWindow* PlotManager::getFigure(const std::string& name) {
    std::string fig = name.empty() ? active_ : name;
    if (fig.empty())
        fig = new_name();
    auto f = fmap_.find(fig);
    PlotWindow* ret;
    if (f == fmap_.end()) {
        ret = new PlotWindow(fig.c_str(), parent_);
        fmap_[fig] = ret;
    }
    else
        ret = f->second;
    if (active_.empty())
        active_ = fig;
    ret->show();
    return ret;
}

// select / create figure and return name of figure
bool PlotManager::setFigure(const std::string& name) {
    if (name.empty())
        return false;
    auto f = fmap_.find(name);
    PlotWindow* pw;
    if (f == fmap_.end()) {
        pw = new PlotWindow(name.c_str(), parent_);
        fmap_[name] = pw;
    }
    else
        pw = f->second;
    active_ = name;
    pw->show();
    return true;
}

// close figure and reset plot
void PlotManager::close(const std::string& name) {
    std::string fig = name.empty() ? active_ : name;
    if (fig.empty())
        return;

    auto f = fmap_.find(fig);
    if (f == fmap_.end())
        return;
    f->second->close();
    delete f->second;
    fmap_.erase(f);
}

void PlotManager::closeAll() {
    for_each(fmap_.begin(), fmap_.end(), [](decltype(*fmap_.begin())& fig){
        fig.second->close();
        delete fig.second;
    });
    fmap_.clear();
}



