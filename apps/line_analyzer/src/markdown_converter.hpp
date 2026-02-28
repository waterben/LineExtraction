/// @file markdown_converter.hpp
/// @brief Lightweight Markdown-to-HTML converter for help viewer.
///
/// Supports the Markdown subset used in the project READMEs:
/// headings, bold/italic, inline code, code blocks, links, lists,
/// tables (GFM), blockquotes, horizontal rules, and paragraphs.

#pragma once

#include <algorithm>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace lsfm {

/// @brief Convert a Markdown string to HTML.
///
/// Handles the subset of Markdown used in the project documentation.
/// Each heading gets an `id` anchor derived from its text for deep-linking.
///
/// @param md  The raw Markdown source.
/// @return HTML string suitable for QTextBrowser.
inline std::string markdownToHtml(const std::string& md) {
  std::istringstream stream(md);
  std::ostringstream html;
  std::string line;

  // State tracking.
  bool in_code_block = false;
  bool in_list = false;
  bool in_table = false;
  bool in_blockquote = false;
  bool need_paragraph_close = false;

  // Close any open paragraph.
  auto closeParagraph = [&]() {
    if (need_paragraph_close) {
      html << "</p>\n";
      need_paragraph_close = false;
    }
  };

  // Close an open list.
  auto closeList = [&]() {
    if (in_list) {
      html << "</ul>\n";
      in_list = false;
    }
  };

  // Close an open table.
  auto closeTable = [&]() {
    if (in_table) {
      html << "</tbody></table>\n";
      in_table = false;
    }
  };

  // Close an open blockquote.
  auto closeBlockquote = [&]() {
    if (in_blockquote) {
      html << "</blockquote>\n";
      in_blockquote = false;
    }
  };

  // Replace inline formatting in a line of text.
  auto inlineFormat = [](const std::string& text) -> std::string {
    std::string out = text;

    // Images: ![alt](src) — render as <img> tag.
    {
      static const std::regex re(R"(!\[([^\]]*)\]\(([^)]+)\))");
      out = std::regex_replace(out, re, R"(<img src="$2" alt="$1" />)");
    }

    // Links: [text](url)
    {
      static const std::regex re(R"(\[([^\]]+)\]\(([^)]+)\))");
      out = std::regex_replace(out, re, R"(<a href="$2">$1</a>)");
    }

    // Inline code: `code`
    {
      static const std::regex re(R"(`([^`]+)`)");
      out = std::regex_replace(out, re, R"(<code>$1</code>)");
    }

    // Bold + italic: ***text*** or ___text___
    {
      static const std::regex re(R"(\*{3}([^*]+)\*{3})");
      out = std::regex_replace(out, re, R"(<b><i>$1</i></b>)");
    }

    // Bold: **text** or __text__
    {
      static const std::regex re(R"(\*{2}([^*]+)\*{2})");
      out = std::regex_replace(out, re, R"(<b>$1</b>)");
    }

    // Italic: *text* or _text_
    {
      static const std::regex re(R"(\*([^*]+)\*)");
      out = std::regex_replace(out, re, R"(<i>$1</i>)");
    }

    // Em dash: ---  (only when surrounded by spaces or at start/end)
    // Skip — the triple-dash is handled as horizontal rule at block level.

    return out;
  };

  // Generate a slug for heading anchors (lowercase, dashes for spaces/special).
  auto slugify = [](const std::string& text) -> std::string {
    std::string slug;
    for (char ch : text) {
      if (std::isalnum(static_cast<unsigned char>(ch))) {
        slug += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
      } else if (ch == ' ' || ch == '-' || ch == '_') {
        if (!slug.empty() && slug.back() != '-') slug += '-';
      }
      // Skip other characters (punctuation, etc.).
    }
    // Trim trailing dash.
    while (!slug.empty() && slug.back() == '-') slug.pop_back();
    return slug;
  };

  // Strip markdown formatting to get plain text (for slugify).
  auto stripFormatting = [](const std::string& text) -> std::string {
    std::string out = text;
    // Remove link syntax: [text](url) → text
    {
      static const std::regex re(R"(\[([^\]]+)\]\([^)]+\))");
      out = std::regex_replace(out, re, "$1");
    }
    // Remove inline code backticks.
    {
      static const std::regex re(R"(`)");
      out = std::regex_replace(out, re, "");
    }
    // Remove bold/italic markers.
    {
      static const std::regex re(R"(\*{1,3})");
      out = std::regex_replace(out, re, "");
    }
    return out;
  };

  html << R"(<html><head><style>
body { font-family: sans-serif; font-size: 10pt; margin: 12px; }
h1 { font-size: 16pt; border-bottom: 2px solid #ccc; padding-bottom: 4px; }
h2 { font-size: 13pt; border-bottom: 1px solid #ddd; padding-bottom: 3px; margin-top: 18px; }
h3 { font-size: 11pt; margin-top: 14px; }
h4 { font-size: 10pt; margin-top: 10px; }
code { background: #f0f0f0; padding: 1px 4px; border-radius: 3px; font-size: 9pt; }
pre { background: #f4f4f4; padding: 8px; border-radius: 4px; font-size: 9pt;
      white-space: pre-wrap; word-wrap: break-word; overflow-x: auto; }
pre code { background: none; padding: 0; }
table { border-collapse: collapse; margin: 8px 0; width: 100%; }
th, td { border: 1px solid #ccc; padding: 4px 8px; text-align: left; }
th { background: #f0f0f0; font-weight: bold; }
blockquote { border-left: 3px solid #ccc; margin: 8px 0; padding: 4px 12px; color: #555; }
a { color: #0366d6; }
hr { border: none; border-top: 1px solid #ccc; margin: 12px 0; }
ul { margin: 4px 0 4px 20px; }
img { max-width: 100%; }
</style></head><body>
)";

  while (std::getline(stream, line)) {
    // --- Fenced code blocks ---
    if (line.substr(0, 3) == "```") {
      if (!in_code_block) {
        closeParagraph();
        closeList();
        closeTable();
        closeBlockquote();
        html << "<pre><code>";
        in_code_block = true;
      } else {
        html << "</code></pre>\n";
        in_code_block = false;
      }
      continue;
    }
    if (in_code_block) {
      // Escape HTML inside code blocks.
      std::string escaped;
      for (char ch : line) {
        switch (ch) {
          case '<':
            escaped += "&lt;";
            break;
          case '>':
            escaped += "&gt;";
            break;
          case '&':
            escaped += "&amp;";
            break;
          default:
            escaped += ch;
        }
      }
      html << escaped << "\n";
      continue;
    }

    // --- Blank line ---
    if (line.empty() || line.find_first_not_of(' ') == std::string::npos) {
      closeParagraph();
      closeList();
      closeBlockquote();
      // Don't close table on blank lines (some tables have them).
      continue;
    }

    // --- Horizontal rule ---
    {
      std::string trimmed = line;
      trimmed.erase(0, trimmed.find_first_not_of(' '));
      if (trimmed.size() >= 3 &&
          (trimmed.find_first_not_of('-') == std::string::npos || trimmed.find_first_not_of('*') == std::string::npos ||
           trimmed.find_first_not_of('_') == std::string::npos)) {
        // Check it's actually all the same char.
        bool is_hr = true;
        char hr_char = trimmed[0];
        if (hr_char == '-' || hr_char == '*' || hr_char == '_') {
          for (char ch : trimmed) {
            if (ch != hr_char && ch != ' ') {
              is_hr = false;
              break;
            }
          }
        } else {
          is_hr = false;
        }
        if (is_hr) {
          closeParagraph();
          closeList();
          closeTable();
          closeBlockquote();
          html << "<hr />\n";
          continue;
        }
      }
    }

    // --- Headings ---
    if (line[0] == '#') {
      closeParagraph();
      closeList();
      closeTable();
      closeBlockquote();
      int level = 0;
      while (level < static_cast<int>(line.size()) && line[static_cast<size_t>(level)] == '#') ++level;
      if (level > 6) level = 6;
      std::string text = line.substr(static_cast<size_t>(level));
      // Trim leading spaces.
      text.erase(0, text.find_first_not_of(' '));
      std::string slug = slugify(stripFormatting(text));
      html << "<h" << level << " id=\"" << slug << "\">" << inlineFormat(text) << "</h" << level << ">\n";
      continue;
    }

    // --- Blockquotes ---
    if (line.size() > 1 && line[0] == '>') {
      closeParagraph();
      closeList();
      closeTable();
      if (!in_blockquote) {
        html << "<blockquote>\n";
        in_blockquote = true;
      }
      std::string content = line.substr(1);
      if (!content.empty() && content[0] == ' ') content.erase(0, 1);
      html << inlineFormat(content) << "<br/>\n";
      continue;
    } else if (in_blockquote) {
      closeBlockquote();
    }

    // --- Tables ---
    if (line.find('|') != std::string::npos) {
      // Check if this looks like a table row (starts with | or has multiple |).
      std::string trimmed = line;
      trimmed.erase(0, trimmed.find_first_not_of(' '));

      // Count pipes.
      int pipes = 0;
      for (char ch : trimmed) {
        if (ch == '|') ++pipes;
      }

      if (pipes >= 2) {
        closeParagraph();
        closeList();
        closeBlockquote();

        // Check if this is a separator row (---|---|---).
        bool is_separator = true;
        for (char ch : trimmed) {
          if (ch != '|' && ch != '-' && ch != ':' && ch != ' ') {
            is_separator = false;
            break;
          }
        }

        if (is_separator && in_table) {
          // Skip the separator row — already emitted thead.
          continue;
        }

        // Parse cells.
        std::vector<std::string> cells;
        std::istringstream cell_stream(trimmed);
        std::string cell;
        bool first = true;
        while (std::getline(cell_stream, cell, '|')) {
          if (first && cell.empty()) {
            first = false;
            continue;  // Skip leading empty cell from leading |.
          }
          first = false;
          // Trim whitespace.
          auto start = cell.find_first_not_of(' ');
          auto end = cell.find_last_not_of(' ');
          if (start != std::string::npos) {
            cells.push_back(cell.substr(start, end - start + 1));
          } else {
            cells.push_back("");
          }
        }
        // Remove trailing empty cell from trailing |.
        if (!cells.empty() && cells.back().empty()) cells.pop_back();

        if (!in_table) {
          // Start table with header row.
          html << "<table>\n<thead><tr>\n";
          for (const auto& c : cells) {
            html << "  <th>" << inlineFormat(c) << "</th>\n";
          }
          html << "</tr></thead>\n<tbody>\n";
          in_table = true;
        } else {
          // Body row.
          html << "<tr>\n";
          for (const auto& c : cells) {
            html << "  <td>" << inlineFormat(c) << "</td>\n";
          }
          html << "</tr>\n";
        }
        continue;
      }
    }
    // If we were in a table and this line is not a table row, close it.
    if (in_table) closeTable();

    // --- Unordered lists ---
    if ((line.size() > 2 && (line[0] == '-' || line[0] == '*' || line[0] == '+') && line[1] == ' ') ||
        (line.size() > 4 && line.substr(0, 2) == "  " && (line[2] == '-' || line[2] == '*' || line[2] == '+') &&
         line[3] == ' ')) {
      closeParagraph();
      closeTable();
      closeBlockquote();
      if (!in_list) {
        html << "<ul>\n";
        in_list = true;
      }
      // Strip the bullet marker.
      std::string content = line;
      content.erase(0, content.find_first_not_of(' '));
      if (!content.empty()) content.erase(0, 1);  // Remove -/*/+
      if (!content.empty() && content[0] == ' ') content.erase(0, 1);
      html << "  <li>" << inlineFormat(content) << "</li>\n";
      continue;
    }
    // If we were in a list and this is not a list item, close it.
    if (in_list) closeList();

    // --- Normal paragraph text ---
    if (!need_paragraph_close) {
      html << "<p>";
      need_paragraph_close = true;
    } else {
      // Continuation of current paragraph (soft line break).
      html << "\n";
    }
    html << inlineFormat(line);
  }

  // Close any remaining open elements.
  closeParagraph();
  closeList();
  closeTable();
  closeBlockquote();

  html << "</body></html>\n";
  return html.str();
}

/// @brief Extract a table-of-contents from Markdown source.
///
/// Returns a vector of (level, text, anchor_slug) tuples for each heading.
struct TocEntry {
  int level;           ///< Heading level (1–6).
  std::string text;    ///< Plain heading text.
  std::string anchor;  ///< HTML anchor id (slugified).
};

inline std::vector<TocEntry> extractToc(const std::string& md) {
  std::istringstream stream(md);
  std::string line;
  std::vector<TocEntry> toc;
  bool in_code_block = false;

  auto slugify = [](const std::string& text) -> std::string {
    std::string slug;
    for (char ch : text) {
      if (std::isalnum(static_cast<unsigned char>(ch))) {
        slug += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
      } else if (ch == ' ' || ch == '-' || ch == '_') {
        if (!slug.empty() && slug.back() != '-') slug += '-';
      }
    }
    while (!slug.empty() && slug.back() == '-') slug.pop_back();
    return slug;
  };

  auto stripFormatting = [](const std::string& text) -> std::string {
    std::string out = text;
    {
      static const std::regex re(R"(\[([^\]]+)\]\([^)]+\))");
      out = std::regex_replace(out, re, "$1");
    }
    {
      static const std::regex re(R"(`)");
      out = std::regex_replace(out, re, "");
    }
    {
      static const std::regex re(R"(\*{1,3})");
      out = std::regex_replace(out, re, "");
    }
    return out;
  };

  while (std::getline(stream, line)) {
    if (line.substr(0, 3) == "```") {
      in_code_block = !in_code_block;
      continue;
    }
    if (in_code_block) continue;

    if (!line.empty() && line[0] == '#') {
      int level = 0;
      while (level < static_cast<int>(line.size()) && line[static_cast<size_t>(level)] == '#') ++level;
      std::string text = line.substr(static_cast<size_t>(level));
      text.erase(0, text.find_first_not_of(' '));
      std::string plain = stripFormatting(text);
      toc.push_back({level, plain, slugify(plain)});
    }
  }
  return toc;
}

}  // namespace lsfm
