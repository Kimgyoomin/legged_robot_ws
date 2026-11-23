# navigation_dds

납품의 설정·실행 프로토콜은 [docs/DDS.md](../../docs/DDS.md)에 정리되어 있습니다.

- CLI: `ros2 run navigation_dds with_profile common <command...>` (`common` 또는 `large`)
- Native launch: `dds_profile:=navigation` 또는 `default`
- XML: `config/fastdds_common.xml`, `config/fastdds_large.xml`
- `default`는 상속한 환경을 그대로 유지합니다.
