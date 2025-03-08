import re
import sys
import shutil
import unittest


def replace_underscores_in_tex(file_path):
    # Define regex pattern to match \label{...} and \ref{...}
    pattern = re.compile(r"(\\label\{([^}]*)\})|(\\ref\{([^}]*)\})")

    # Read the content of the file
    with open(file_path, "r", encoding="utf-8") as file:
        content = file.read()

    # Apply the regex replacement
    def replace_match(m):
        if m.group(1):  # \\label case
            return f"\\label{{{m.group(2).replace('_', '-')}}}"
        elif m.group(3):  # \\ref case
            return f"\\ref{{{m.group(4).replace('_', '-')}}}"
        return m.group(0)

    updated_content, num_replacements = pattern.subn(replace_match, content)

    if num_replacements > 0:
        # Create a backup before modifying
        shutil.copy(file_path, file_path + ".bak")

        # Write back the modified content
        with open(file_path, "w", encoding="utf-8") as file:
            file.write(updated_content)

        print(f"Processed file: {file_path}")
        print(f"Number of replacements made: {num_replacements}")
    else:
        print("No matches found. File was not modified.")


class TestReplaceUnderscores(unittest.TestCase):
    def test_replacement(self):
        test_cases = {
            "\\label{test_case_1}": "\\label{test-case-1}",
            "\\ref{example_test_2}": "\\ref{example-test-2}",
            "Some text without change": "Some text without change",
            "\\label{no_underscores_here}": "\\label{no-underscores-here}",
            "\\ref{another_test_case_example}": "\\ref{another-test-case-example}",
        }

        pattern = re.compile(r"(\\label\{([^}]*)\})|(\\ref\{([^}]*)\})")

        def replace_match(m):
            if m.group(1):  # \\label case
                return f"\\label{{{m.group(2).replace('_', '-')}}}"
            elif m.group(3):  # \\ref case
                return f"\\ref{{{m.group(4).replace('_', '-')}}}"
            return m.group(0)

        for input_text, expected_output in test_cases.items():
            result = pattern.sub(replace_match, input_text)
            self.assertEqual(result, expected_output)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] != "test":
        replace_underscores_in_tex(sys.argv[1])
    else:
        unittest.main(argv=[sys.argv[0]], exit=False)
