# insert_vps_result
cam_subscriber から MQTT で送られてくる VPS 結果を Subscribe して、  
データベースを更新するスクリプト

# 実行方法
トピック名が vsp_result の場合
```
python3 mqtt_sub_insert_db.py vps_result
```

# DB操作テストスクリプト
**db_test_script/create.py**  
テーブル作成  

**db_test_script/insert.py**  
データ挿入テスト  
