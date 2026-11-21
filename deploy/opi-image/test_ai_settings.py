import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import ai_settings


class SavedCredentialsTest(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        config = patch.object(ai_settings, "CONFIG", Path(folder.name))
        config.start()
        self.addCleanup(config.stop)

    def test_switching_provider_restores_its_key(self):
        legacy = {"model": {"provider": "zhipu", "base_url": ai_settings.PRESETS["zhipu"][0],
                            "model": "glm-4.6v-flash", "api_key": "test-glm"}}
        (ai_settings.CONFIG / "ai-settings.json").write_text(json.dumps(legacy), encoding="utf-8")
        ai_settings.save({"model": {"provider": "dashscope", "api_key": "test-qwen"}})
        ai_settings.save({"voice_mode": "duck"})
        ai_settings.save({"model": {"provider": "zhipu"}})
        self.assertEqual(ai_settings.api_key(ai_settings.read()["model"]), "test-glm")
        ai_settings.save({"model": {"provider": "dashscope"}})
        self.assertEqual(ai_settings.api_key(ai_settings.read()["model"]), "test-qwen")
        public = ai_settings.public()
        self.assertTrue(public["model"]["has_key"])
        self.assertNotIn("credentials", public)
        self.assertNotIn("api_key", public["model"])

    def test_cleared_key_stays_cleared_after_switching(self):
        (ai_settings.CONFIG / "bigmodel.key").write_text("test-legacy-key")
        ai_settings.save({"model": {"provider": "zhipu", "clear_key": True}})
        ai_settings.save({"model": {"provider": "dashscope"}})
        ai_settings.save({"model": {"provider": "zhipu"}})
        self.assertFalse(ai_settings.public()["model"]["has_key"])

    def test_custom_endpoints_and_voice_credentials_are_separate(self):
        first = {"provider": "compatible", "base_url": "https://one.example/v1", "model": "test"}
        second = {**first, "base_url": "https://two.example/v1"}
        ai_settings.save({"model": {**first, "api_key": "test-chat"},
                          "tts": {**first, "api_key": "test-voice", "voice": "test"}})
        ai_settings.save({"model": second, "tts": {"provider": "edge", "voice": "test"}})
        self.assertFalse(ai_settings.public()["model"]["has_key"])
        ai_settings.save({"model": first, "tts": {**first, "voice": "test"}})
        actual = ai_settings.read()
        self.assertEqual(ai_settings.api_key(actual["model"]), "test-chat")
        self.assertEqual(ai_settings.api_key(actual["tts"]), "test-voice")


if __name__ == "__main__":
    unittest.main()
