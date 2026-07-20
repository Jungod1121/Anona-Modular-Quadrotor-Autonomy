"""Unit tests for maps_catalog metadata and homemade pose sanity."""

from __future__ import annotations

import unittest

from drone_bringup.maps_catalog import (
    DIFFICULTIES,
    MAPS,
    catalog_metadata,
    homemade_connectivity_sanity,
    normalize_map_id,
)


class TestMapsCatalog(unittest.TestCase):
    def test_every_map_has_seed_and_difficulty(self) -> None:
        required = (
            'difficulty', 'seed', 'obstacle_family', 'safety_radius', 'bounds',
        )
        for map_id, meta in MAPS.items():
            for key in required:
                self.assertIn(key, meta, msg=f'{map_id} missing {key}')
            self.assertIn(meta['difficulty'], DIFFICULTIES, msg=map_id)
            self.assertIsInstance(meta['seed'], int, msg=map_id)
            bounds = meta['bounds']
            for axis in ('xmin', 'xmax', 'ymin', 'ymax', 'zmin', 'zmax'):
                self.assertIn(axis, bounds, msg=f'{map_id} bounds.{axis}')
            self.assertLess(bounds['xmin'], bounds['xmax'], msg=map_id)
            self.assertLess(bounds['ymin'], bounds['ymax'], msg=map_id)
            self.assertLessEqual(bounds['zmin'], bounds['zmax'], msg=map_id)

    def test_catalog_metadata_seed_override(self) -> None:
        meta = catalog_metadata('sparse', seed=99)
        self.assertEqual(meta['seed'], 99)
        self.assertEqual(meta['difficulty'], 'simple')
        self.assertEqual(meta['id'], 'sparse')

    def test_catalog_metadata_default_seed_is_deterministic(self) -> None:
        a = catalog_metadata('official_maze2d')
        b = catalog_metadata('official_maze2d')
        self.assertEqual(a['seed'], b['seed'])
        self.assertEqual(a['seed'], MAPS['official_maze2d']['seed'])

    def test_tier_presets_exist_and_alias(self) -> None:
        tiers = (
            'tier_simple_open',
            'tier_medium_corridor',
            'tier_complex_forest',
            'tier_extreme_maze',
        )
        for tid in tiers:
            self.assertIn(tid, MAPS)
        self.assertEqual(normalize_map_id('tier_simple'), 'tier_simple_open')
        self.assertEqual(normalize_map_id('extreme_maze'), 'tier_extreme_maze')

    def test_legacy_map_ids_unchanged(self) -> None:
        legacy = (
            'dense_field', 'sparse', 'narrow_corridor', 'ego_maze2d_port',
            'ego_forest_port', 'official_forest', 'official_perlin',
            'official_posts', 'official_maze2d', 'official_maze3d',
        )
        for map_id in legacy:
            self.assertIn(map_id, MAPS)
            self.assertNotIn('based_on', MAPS[map_id])

    def test_homemade_connectivity_sanity(self) -> None:
        for map_id, meta in MAPS.items():
            if meta['family'] != 'homemade':
                continue
            report = homemade_connectivity_sanity(map_id)
            self.assertFalse(report.get('skipped'), msg=map_id)
            self.assertTrue(report['ok'], msg=f'{map_id}: {report}')


if __name__ == '__main__':
    unittest.main()
