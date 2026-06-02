import os
import sys

# 노드/모듈이 있는 상위 폴더(patch(20260602))를 import 경로에 추가.
# (폴더명에 공백/괄호가 있어도 동작하도록 절대경로 사용)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
