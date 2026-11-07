"""RES-507 plot coordinates may crop a view but must not alter data values."""
import unittest
import xml.etree.ElementTree as ET
from scripts.svgplot import Figure


class PlotFidelityTests(unittest.TestCase):
    def test_line_retains_out_of_range_data_coordinate_under_svg_clip(self):
        figure = Figure(200, 200, "manufactured", "no empirical claim")
        panel = figure.panel(20, 20, 100, 100, (0, 1), (0, 1))
        panel.line([0, 1], [0.5, 10], "#000000")
        root = ET.fromstring(figure.render())
        line = next(e for e in root.iter() if e.tag.endswith("polyline"))
        last_y = float(line.attrib["points"].split()[-1].split(",")[1])
        self.assertEqual(last_y, 20 + 100 - 10 * 100)
        self.assertTrue(any(e.tag.endswith("clipPath") for e in root.iter()))

    def test_marker_retains_out_of_range_data_coordinate(self):
        figure = Figure(200, 200, "manufactured", "no empirical claim")
        panel = figure.panel(20, 20, 100, 100, (0, 1), (0, 1))
        panel.markers([0.5], [10], "#000000")
        root = ET.fromstring(figure.render())
        marker = next(e for e in root.iter() if e.tag.endswith("circle"))
        self.assertEqual(float(marker.attrib["cy"]), 20 + 100 - 10 * 100)
