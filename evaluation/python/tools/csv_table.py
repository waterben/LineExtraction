import argparse
import pandas as pd
from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle

def get_table_style(style_name, first_col_names):
    styles = {
        "simple": TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.grey),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.whitesmoke),
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
            ('FONTSIZE', (0, 0), (-1, -1), 10),
            ('BOTTOMPADDING', (0, 0), (-1, 0), 12),
            ('BACKGROUND', (0, 1), (-1, -1), colors.beige),
            ('GRID', (0, 0), (-1, -1), 1, colors.black),
        ]),
        "scientific": TableStyle([
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('FONTNAME', (0, 0), (-1, -1), 'Times-Roman'),
            ('FONTSIZE', (0, 0), (-1, -1), 10),
            ('LINEBELOW', (0, 0), (-1, 0), 1, colors.black),  # Line below header
            ('LINEABOVE', (0, 0), (-1, 0), 1, colors.black),  # Line above header
            ('LINEBELOW', (0, -1), (-1, -1), 1, colors.black),  # Line below table
            ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 6),
        ])
    }
    
    style = styles.get(style_name, styles["simple"])
    if first_col_names:
        style.add('ALIGN', (0, 1), (0, -1), 'LEFT')
    return style

def csv_to_pdf(input_csv, output_pdf, sep=';', header=True, first_col_names=True, precision=3, style="simple"):
    # Read CSV file
    df = pd.read_csv(input_csv, sep=sep, header=0 if header else None)
    
    # Format numerical values
    df = df.round(precision)
    
    # Convert DataFrame to list of lists
    data = [df.columns.tolist()] + df.values.tolist() if header else df.values.tolist()
    
    # Create PDF document
    pdf = SimpleDocTemplate(output_pdf, pagesize=letter)
    table = Table(data)
    
    # Apply chosen style
    table.setStyle(get_table_style(style, first_col_names))
    
    elements = [table]
    pdf.build(elements)
    print(f"PDF saved as {output_pdf}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert CSV to PDF table.")
    parser.add_argument("input_csv", help="Path to input CSV file")
    parser.add_argument("output_pdf", help="Path to output PDF file")
    parser.add_argument("--sep", default=";", help="CSV separator (default: ';')")
    parser.add_argument("--header", type=bool, default=True, help="Does CSV have a header row? (default: True)")
    parser.add_argument("--first_col_names", type=bool, default=True, help="Is the first column representing names? (default: True)")
    parser.add_argument("--precision", type=int, default=3, help="Precision for numerical values (default: 3 digits)")
    parser.add_argument("--style", default="simple", help="Table style ('simple' or 'scientific', default: 'simple')")
    
    args = parser.parse_args()
    
    csv_to_pdf(args.input_csv, args.output_pdf, args.sep, args.header, args.first_col_names, args.precision, args.style)
