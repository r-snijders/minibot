import base64
import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer
from minibot.ai_protocol import ObservationMemory, infer, validate_observation


class ProtocolTests(unittest.TestCase):
    def test_strict_boolean_and_description(self):
        for value in ({}, {'person_visible': 'false', 'description': 'room'},
                      {'person_visible': True, 'description': None}, []):
            with self.assertRaises(ValueError):
                validate_observation(value)

    def test_expiry_accounts_for_inference_latency(self):
        memory = ObservationMemory(ttl=15)
        memory.accept({'person_visible': True, 'description': 'person', 'age_seconds': 12}, 100)
        self.assertTrue(memory.visible(102.9))
        self.assertFalse(memory.visible(103))
        for age in (15, -1, float('nan'), '1', True):
            with self.assertRaises(ValueError):
                memory.accept({'person_visible': True, 'description': '', 'age_seconds': age}, 100)

    def test_negative_observation_clears_person(self):
        memory = ObservationMemory()
        memory.accept({'person_visible': True, 'description': '', 'age_seconds': 0}, 0)
        memory.accept({'person_visible': False, 'description': '', 'age_seconds': 0}, 1)
        self.assertFalse(memory.visible(2))

    def test_ollama_http_contract(self):
        captured = {}
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                captured.update(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
                captured['path'] = self.path
                response = json.dumps({'done': True, 'response': json.dumps({
                    'person_visible': True, 'description': 'A person near a door'})}).encode()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(response)
            def log_message(self, *args):
                pass
        server = HTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            image = base64.b64encode(b'image bytes').decode()
            result = infer(f'http://127.0.0.1:{server.server_port}', 'vision-model', image, 2)
            self.assertTrue(result['person_visible'])
            self.assertEqual(captured['path'], '/api/generate')
            self.assertEqual(captured['images'], [image])
            self.assertFalse(captured['stream'])
            self.assertIsInstance(captured['format'], dict)
        finally:
            server.shutdown()
            server.server_close()
            thread.join()
