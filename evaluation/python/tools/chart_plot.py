import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import pandas as pd
import argparse
import os
import csv


def parse_chart_xml(
    chart_file, default_chart_title, default_category_title, default_values_title, override_title, verbose
):
    """Parse the chart.xml file and extract data dynamically."""
    tree = ET.parse(chart_file)
    root = tree.getroot()
    ns = {"c": "http://schemas.openxmlformats.org/drawingml/2006/chart"}

    if override_title:
        chart_title = default_chart_title
        category_title = default_category_title
        values_title = default_values_title
    else:
        # Extract chart title
        title_element = root.find(".//c:title//c:tx//c:rich//c:p//c:r//c:t", ns)
        chart_title = title_element.text if title_element is not None else default_chart_title

        # Extract category axis title
        category_title_element = root.find(".//c:catAx//c:title//c:tx//c:rich//c:p//c:r//c:t", ns)
        category_title = category_title_element.text if category_title_element is not None else default_category_title

        # Extract values axis title
        values_title_element = root.find(".//c:valAx//c:title//c:tx//c:rich//c:p//c:r//c:t", ns)
        values_title = values_title_element.text if values_title_element is not None else default_values_title

    # Extract categories (X-axis labels), ensuring uniqueness
    categories = []
    cat_elements = root.findall(".//c:cat/c:strRef/c:strCache/c:pt", ns)
    for cat in cat_elements:
        category = cat.find("c:v", ns).text
        if category not in categories:
            categories.append(category)

    # Extract data series
    data_series = {}
    series_names = []
    for series in root.findall(".//c:ser", ns):
        # Series name
        series_name_element = series.find(".//c:tx//c:strRef//c:strCache//c:pt//c:v", ns)
        series_name = series_name_element.text if series_name_element is not None else "Series"

        # Extract data points
        values = []
        value_elements = series.findall(".//c:val/c:numRef/c:numCache/c:pt", ns)
        for val in value_elements:
            values.append(float(val.find("c:v", ns).text))

        data_series[series_name] = values
        series_names.append(series_name)

    if verbose:
        print(f"Extracted chart title: {chart_title}")
        print(f"Extracted category title: {category_title}")
        print("Extracted categories:")
        print(categories)
        print(f"Extracted values title: {values_title}")
        print("Extracted series:")
        print(series_names)

    return chart_title, category_title, values_title, categories, data_series


def parse_csv(
    csv_file, separator, default_chart_title, default_category_title, default_values_title, override_title, verbose
):
    """Parse the CSV file and extract data."""
    df = pd.read_csv(csv_file, sep=separator)

    if override_title:
        category_title = default_category_title
    else:
        category_title = df.columns[0] if df.columns[0].strip() else default_category_title

    categories = df.iloc[:, 0].tolist()

    series_names = df.columns[1:].tolist()  # Extract series names from first row (excluding first column)
    data_series = {series_names[i]: df.iloc[0:, i + 1].tolist() for i in range(len(series_names))}  # Map names to data

    if verbose:
        print(f"Extracted chart title: {default_chart_title}")
        print(f"Extracted category title: {category_title}")
        print("Extracted categories:")
        print(categories)
        print(f"Extracted values title: {default_values_title}")
        print("Extracted series:")
        print(series_names)

    return default_chart_title, category_title, default_values_title, categories, data_series


def generate_csv(output_csv, category_title, categories, data_series, separator=","):

    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, delimiter=separator)

        # Write header
        writer.writerow([category_title] + list(data_series.keys()))

        # Write data rows
        for i in range(len(categories)):
            row = [categories[i]] + [data_series[series][i] for series in data_series]
            writer.writerow(row)


def generate_bar_chart(
    chart_title, category_title, values_title, categories, data_series, output_file, as_line=None, show_plot=False
):
    """Generate the chart and save it as a PDF."""
    df = pd.DataFrame(data_series, index=categories)

    plt.figure(figsize=(12, 6))
    df_without_line = df.drop(columns=[as_line], errors="ignore") if as_line else df
    colors = plt.colormaps.get_cmap("tab10")
    bar_colors = [colors(i) for i in range(len(df_without_line.columns))]
    df_without_line.plot(kind="bar", edgecolor="black", ax=plt.gca(), color=bar_colors)

    # Plot specified series as a line if given
    if as_line and as_line in df.columns:
        line_color = colors(len(df_without_line.columns) % 10)  # Ensure a distinct color for the line
        plt.plot(df.index, df[as_line], linestyle="-", linewidth=2, label=as_line, color=line_color)

    plt.title(chart_title, fontsize=14, fontweight="bold")
    plt.xlabel(category_title, fontsize=12, fontweight="bold")
    plt.ylabel(values_title, fontsize=12, fontweight="bold")
    plt.legend(title="Series", loc="upper left", fontsize=10)
    plt.xticks(rotation=45, ha="right")
    plt.grid(axis="y", linestyle="--", linewidth=0.5)

    plt.savefig(output_file, bbox_inches="tight")
    if show_plot:
        plt.show()


def generate_line_chart(
    chart_title, category_title, values_title, categories, data_series, output_file, show_plot=False
):
    """Generate the chart and save it as a PDF."""
    df = pd.DataFrame(data_series, index=categories)

    plt.figure(figsize=(12, 6))
    colors = plt.colormaps.get_cmap("tab10")
    line_colors = [colors(i) for i in range(len(df.columns))]
    markers = ["o", "s", "D", "^", "v", "p", "*", "X", "<", ">", "h", "+"]

    for i, column in enumerate(df.columns):
        plt.plot(
            df.index,
            df[column],
            marker=markers[i % len(markers)],
            linestyle="-",
            linewidth=2,
            label=column,
            color=line_colors[i],
        )

    plt.title(chart_title, fontsize=14, fontweight="bold")
    plt.xlabel(category_title, fontsize=12, fontweight="bold")
    plt.ylabel(values_title, fontsize=12, fontweight="bold")
    plt.legend(title="Series", loc="upper left", fontsize=10)
    plt.xticks(rotation=45, ha="right")
    plt.grid(axis="y", linestyle="--", linewidth=0.5)

    plt.savefig(output_file, bbox_inches="tight")
    if show_plot:
        plt.show()


def main(
    input_file,
    output_file,
    chart_title,
    category_title,
    values_title,
    override_title,
    mode,
    separator,
    as_line,
    show_plot,
    verbose,
):
    """Main function to process the XML or CSV file and generate the chart."""
    in_file_extension = os.path.splitext(input_file)[1].lower()
    if in_file_extension == ".csv":
        if verbose:
            print("CSV file detected. Parsing file...")
        chart_title, category_title, values_title, categories, data_series = parse_csv(
            input_file, separator, chart_title, category_title, values_title, override_title, verbose
        )
    elif in_file_extension == ".xml":
        if verbose:
            print("XML file detected. Parsing file...")
        chart_title, category_title, values_title, categories, data_series = parse_chart_xml(
            input_file, chart_title, category_title, values_title, override_title, verbose
        )
    else:
        raise ValueError("Unsupported file format. Please provide a .csv or .xml file.")

    out_file_extension = os.path.splitext(output_file)[1].lower()
    if out_file_extension == ".pdf":
        if mode == "bar":
            if verbose:
                print("Generate bar chart...")
            generate_bar_chart(
                chart_title, category_title, values_title, categories, data_series, output_file, as_line, show_plot
            )
        elif mode == "line":
            if verbose:
                print("Generate line chart...")
            generate_line_chart(
                chart_title, category_title, values_title, categories, data_series, output_file, show_plot
            )
        else:
            raise ValueError("Unsupported chart plot mode. Please provide a valid mode (bar, line).")
    elif out_file_extension == ".csv":
        if verbose:
            print("Save as CSV...")
        generate_csv(output_file, category_title, categories, data_series, separator)
    else:
        raise ValueError("Unsupported output format. Please provide a .csv or .pdf file.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert Word Chart XML or CSV to PDF Chart or XML to CSV.")
    parser.add_argument("input_file", help="Path to chart.xml or CSV file")
    parser.add_argument("output_file", help="Path to output PDF or CSV file")
    parser.add_argument("--chart_title", help="Title of the chart", default="Chart")
    parser.add_argument("--category_title", help="Title of the category axis", default="Categories")
    parser.add_argument("--values_title", help="Title of the values axis", default="Values")
    parser.add_argument("--override_title", help="Force override extracted titles by passed in titles", default=None)
    parser.add_argument("--mode", help="Chart mode: line or bar (default is bar)", default="bar")
    parser.add_argument("--as_line", help="Series name to be drawn as a line instead of bars", default=None)
    parser.add_argument("--separator", help="CSV separator (default is ',')", default=",")
    parser.add_argument("--show_plot", help="Show generated plot on screen", default=False)
    parser.add_argument("--verbose", help="Verbose mode", default=False)

    args = parser.parse_args()
    main(
        args.input_file,
        args.output_file,
        args.chart_title,
        args.category_title,
        args.values_title,
        args.override_title,
        args.mode,
        args.separator,
        args.as_line,
        args.show_plot,
        args.verbose,
    )
