# UAV

----

​																																												***Author：Husky*** 

## 姿态角

​	姿态角又称为欧拉角，在飞机（载体）上建立物体坐标系X-Y-Z。滚转(Roll)、俯仰(Pitch)和偏航(Yaw)来表示，分别表示飞机绕Y轴、X轴和Z轴旋转。

![img](https://img-blog.csdnimg.cn/20210917101246247.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAUXVhcmsgU3Rhcg==,size_20,color_FFFFFF,t_70,g_se,x_16)





**如图：**

![下载](/Users/husky/Downloads/下载.png)



图中有两组坐标：

- ![xyz](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%223.573ex%22%20height%3D%222.009ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.671ex%3B%22%20viewBox%3D%220%20-576.1%201538.5%20865.1%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3Exyz%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-78%22%20d%3D%22M52%20289Q59%20331%20106%20386T222%20442Q257%20442%20286%20424T329%20379Q371%20442%20430%20442Q467%20442%20494%20420T522%20361Q522%20332%20508%20314T481%20292T458%20288Q439%20288%20427%20299T415%20328Q415%20374%20465%20391Q454%20404%20425%20404Q412%20404%20406%20402Q368%20386%20350%20336Q290%20115%20290%2078Q290%2050%20306%2038T341%2026Q378%2026%20414%2059T463%20140Q466%20150%20469%20151T485%20153H489Q504%20153%20504%20145Q504%20144%20502%20134Q486%2077%20440%2033T333%20-11Q263%20-11%20227%2052Q186%20-10%20133%20-10H127Q78%20-10%2057%2016T35%2071Q35%20103%2054%20123T99%20143Q142%20143%20142%20101Q142%2081%20130%2066T107%2046T94%2041L91%2040Q91%2039%2097%2036T113%2029T132%2026Q168%2026%20194%2071Q203%2087%20217%20139T245%20247T261%20313Q266%20340%20266%20352Q266%20380%20251%20392T217%20404Q177%20404%20142%20372T93%20290Q91%20281%2088%20280T72%20278H58Q52%20284%2052%20289Z%22%3E%3C%2Fpath%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-79%22%20d%3D%22M21%20287Q21%20301%2036%20335T84%20406T158%20442Q199%20442%20224%20419T250%20355Q248%20336%20247%20334Q247%20331%20231%20288T198%20191T182%20105Q182%2062%20196%2045T238%2027Q261%2027%20281%2038T312%2061T339%2094Q339%2095%20344%20114T358%20173T377%20247Q415%20397%20419%20404Q432%20431%20462%20431Q475%20431%20483%20424T494%20412T496%20403Q496%20390%20447%20193T391%20-23Q363%20-106%20294%20-155T156%20-205Q111%20-205%2077%20-183T43%20-117Q43%20-95%2050%20-80T69%20-58T89%20-48T106%20-45Q150%20-45%20150%20-87Q150%20-107%20138%20-122T115%20-142T102%20-147L99%20-148Q101%20-153%20118%20-160T152%20-167H160Q177%20-167%20186%20-165Q219%20-156%20247%20-127T290%20-65T313%20-9T321%2021L315%2017Q309%2013%20296%206T270%20-6Q250%20-11%20231%20-11Q185%20-11%20150%2011T104%2082Q103%2089%20103%20113Q103%20170%20138%20262T173%20379Q173%20380%20173%20381Q173%20390%20173%20393T169%20400T158%20404H154Q131%20404%20112%20385T82%20344T65%20302T57%20280Q55%20278%2041%20278H27Q21%20284%2021%20287Z%22%3E%3C%2Fpath%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-7A%22%20d%3D%22M347%20338Q337%20338%20294%20349T231%20360Q211%20360%20197%20356T174%20346T162%20335T155%20324L153%20320Q150%20317%20138%20317Q117%20317%20117%20325Q117%20330%20120%20339Q133%20378%20163%20406T229%20440Q241%20442%20246%20442Q271%20442%20291%20425T329%20392T367%20375Q389%20375%20411%20408T434%20441Q435%20442%20449%20442H462Q468%20436%20468%20434Q468%20430%20463%20420T449%20399T432%20377T418%20358L411%20349Q368%20298%20275%20214T160%20106L148%2094L163%2093Q185%2093%20227%2082T290%2071Q328%2071%20360%2090T402%20140Q406%20149%20409%20151T424%20153Q443%20153%20443%20143Q443%20138%20442%20134Q425%2072%20376%2031T278%20-11Q252%20-11%20232%206T193%2040T155%2057Q111%2057%2076%20-3Q70%20-11%2059%20-11H54H41Q35%20-5%2035%20-2Q35%2013%2093%2084Q132%20129%20225%20214T340%20322Q352%20338%20347%20338Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-78%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%20transform%3D%22translate(572%2C0)%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-79%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%20transform%3D%22translate(1070%2C0)%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-7A%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)为全局坐标，保持不动
- ![XYZ](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%225.434ex%22%20height%3D%222.176ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.338ex%3B%22%20viewBox%3D%220%20-791.3%202339.5%20936.9%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3EXYZ%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-58%22%20d%3D%22M42%200H40Q26%200%2026%2011Q26%2015%2029%2027Q33%2041%2036%2043T55%2046Q141%2049%20190%2098Q200%20108%20306%20224T411%20342Q302%20620%20297%20625Q288%20636%20234%20637H206Q200%20643%20200%20645T202%20664Q206%20677%20212%20683H226Q260%20681%20347%20681Q380%20681%20408%20681T453%20682T473%20682Q490%20682%20490%20671Q490%20670%20488%20658Q484%20643%20481%20640T465%20637Q434%20634%20411%20620L488%20426L541%20485Q646%20598%20646%20610Q646%20628%20622%20635Q617%20635%20609%20637Q594%20637%20594%20648Q594%20650%20596%20664Q600%20677%20606%20683H618Q619%20683%20643%20683T697%20681T738%20680Q828%20680%20837%20683H845Q852%20676%20852%20672Q850%20647%20840%20637H824Q790%20636%20763%20628T722%20611T698%20593L687%20584Q687%20585%20592%20480L505%20384Q505%20383%20536%20304T601%20142T638%2056Q648%2047%20699%2046Q734%2046%20734%2037Q734%2035%20732%2023Q728%207%20725%204T711%201Q708%201%20678%201T589%202Q528%202%20496%202T461%201Q444%201%20444%2010Q444%2011%20446%2025Q448%2035%20450%2039T455%2044T464%2046T480%2047T506%2054Q523%2062%20523%2064Q522%2064%20476%20181L429%20299Q241%2095%20236%2084Q232%2076%20232%2072Q232%2053%20261%2047Q262%2047%20267%2047T273%2046Q276%2046%20277%2046T280%2045T283%2042T284%2035Q284%2026%20282%2019Q279%206%20276%204T261%201Q258%201%20243%201T201%202T142%202Q64%202%2042%200Z%22%3E%3C%2Fpath%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-59%22%20d%3D%22M66%20637Q54%20637%2049%20637T39%20638T32%20641T30%20647T33%20664T42%20682Q44%20683%2056%20683Q104%20680%20165%20680Q288%20680%20306%20683H316Q322%20677%20322%20674T320%20656Q316%20643%20310%20637H298Q242%20637%20242%20624Q242%20619%20292%20477T343%20333L346%20336Q350%20340%20358%20349T379%20373T411%20410T454%20461Q546%20568%20561%20587T577%20618Q577%20634%20545%20637Q528%20637%20528%20647Q528%20649%20530%20661Q533%20676%20535%20679T549%20683Q551%20683%20578%20682T657%20680Q684%20680%20713%20681T746%20682Q763%20682%20763%20673Q763%20669%20760%20657T755%20643Q753%20637%20734%20637Q662%20632%20617%20587Q608%20578%20477%20424L348%20273L322%20169Q295%2062%20295%2057Q295%2046%20363%2046Q379%2046%20384%2045T390%2035Q390%2033%20388%2023Q384%206%20382%204T366%201Q361%201%20324%201T232%202Q170%202%20138%202T102%201Q84%201%2084%209Q84%2014%2087%2024Q88%2027%2089%2030T90%2035T91%2039T93%2042T96%2044T101%2045T107%2045T116%2046T129%2046Q168%2047%20180%2050T198%2063Q201%2068%20227%20171L252%20274L129%20623Q128%20624%20127%20625T125%20627T122%20629T118%20631T113%20633T105%20634T96%20635T83%20636T66%20637Z%22%3E%3C%2Fpath%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-5A%22%20d%3D%22M58%208Q58%2023%2064%2035Q64%2036%20329%20334T596%20635L586%20637Q575%20637%20512%20637H500H476Q442%20637%20420%20635T365%20624T311%20598T266%20548T228%20469Q227%20466%20226%20463T224%20458T223%20453T222%20450L221%20448Q218%20443%20202%20443Q185%20443%20182%20453L214%20561Q228%20606%20241%20651Q249%20679%20253%20681Q256%20683%20487%20683H718Q723%20678%20723%20675Q723%20673%20717%20649Q189%2054%20188%2052L185%2049H274Q369%2050%20377%2051Q452%2060%20500%20100T579%20247Q587%20272%20590%20277T603%20282H607Q628%20282%20628%20271Q547%205%20541%202Q538%200%20300%200H124Q58%200%2058%208Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-58%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%20transform%3D%22translate(852%2C0)%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-59%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%20transform%3D%22translate(1616%2C0)%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-5A%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)为局部坐标，随着物体一起运动

旋转步骤如下：

- 物体绕**全局**的![z](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%221.088ex%22%20height%3D%221.676ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.338ex%3B%22%20viewBox%3D%220%20-576.1%20468.5%20721.6%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3Ez%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-7A%22%20d%3D%22M347%20338Q337%20338%20294%20349T231%20360Q211%20360%20197%20356T174%20346T162%20335T155%20324L153%20320Q150%20317%20138%20317Q117%20317%20117%20325Q117%20330%20120%20339Q133%20378%20163%20406T229%20440Q241%20442%20246%20442Q271%20442%20291%20425T329%20392T367%20375Q389%20375%20411%20408T434%20441Q435%20442%20449%20442H462Q468%20436%20468%20434Q468%20430%20463%20420T449%20399T432%20377T418%20358L411%20349Q368%20298%20275%20214T160%20106L148%2094L163%2093Q185%2093%20227%2082T290%2071Q328%2071%20360%2090T402%20140Q406%20149%20409%20151T424%20153Q443%20153%20443%20143Q443%20138%20442%20134Q425%2072%20376%2031T278%20-11Q252%20-11%20232%206T193%2040T155%2057Q111%2057%2076%20-3Q70%20-11%2059%20-11H54H41Q35%20-5%2035%20-2Q35%2013%2093%2084Q132%20129%20225%20214T340%20322Q352%20338%20347%20338Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-7A%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)轴旋转![\alpha](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%221.488ex%22%20height%3D%221.676ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.338ex%3B%22%20viewBox%3D%220%20-576.1%20640.5%20721.6%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3E%5Calpha%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-3B1%22%20d%3D%22M34%20156Q34%20270%20120%20356T309%20442Q379%20442%20421%20402T478%20304Q484%20275%20485%20237V208Q534%20282%20560%20374Q564%20388%20566%20390T582%20393Q603%20393%20603%20385Q603%20376%20594%20346T558%20261T497%20161L486%20147L487%20123Q489%2067%20495%2047T514%2026Q528%2028%20540%2037T557%2060Q559%2067%20562%2068T577%2070Q597%2070%20597%2062Q597%2056%20591%2043Q579%2019%20556%205T512%20-10H505Q438%20-10%20414%2062L411%2069L400%2061Q390%2053%20370%2041T325%2018T267%20-2T203%20-11Q124%20-11%2079%2039T34%20156ZM208%2026Q257%2026%20306%2047T379%2090L403%20112Q401%20255%20396%20290Q382%20405%20304%20405Q235%20405%20183%20332Q156%20292%20139%20224T121%20120Q121%2071%20146%2049T208%2026Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-3B1%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)角
- 继续绕**自己**的![X](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%221.98ex%22%20height%3D%222.176ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.338ex%3B%22%20viewBox%3D%220%20-791.3%20852.5%20936.9%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3EX%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-58%22%20d%3D%22M42%200H40Q26%200%2026%2011Q26%2015%2029%2027Q33%2041%2036%2043T55%2046Q141%2049%20190%2098Q200%20108%20306%20224T411%20342Q302%20620%20297%20625Q288%20636%20234%20637H206Q200%20643%20200%20645T202%20664Q206%20677%20212%20683H226Q260%20681%20347%20681Q380%20681%20408%20681T453%20682T473%20682Q490%20682%20490%20671Q490%20670%20488%20658Q484%20643%20481%20640T465%20637Q434%20634%20411%20620L488%20426L541%20485Q646%20598%20646%20610Q646%20628%20622%20635Q617%20635%20609%20637Q594%20637%20594%20648Q594%20650%20596%20664Q600%20677%20606%20683H618Q619%20683%20643%20683T697%20681T738%20680Q828%20680%20837%20683H845Q852%20676%20852%20672Q850%20647%20840%20637H824Q790%20636%20763%20628T722%20611T698%20593L687%20584Q687%20585%20592%20480L505%20384Q505%20383%20536%20304T601%20142T638%2056Q648%2047%20699%2046Q734%2046%20734%2037Q734%2035%20732%2023Q728%207%20725%204T711%201Q708%201%20678%201T589%202Q528%202%20496%202T461%201Q444%201%20444%2010Q444%2011%20446%2025Q448%2035%20450%2039T455%2044T464%2046T480%2047T506%2054Q523%2062%20523%2064Q522%2064%20476%20181L429%20299Q241%2095%20236%2084Q232%2076%20232%2072Q232%2053%20261%2047Q262%2047%20267%2047T273%2046Q276%2046%20277%2046T280%2045T283%2042T284%2035Q284%2026%20282%2019Q279%206%20276%204T261%201Q258%201%20243%201T201%202T142%202Q64%202%2042%200Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-58%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)轴（也就是图中的![N](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%222.064ex%22%20height%3D%222.176ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.338ex%3B%22%20viewBox%3D%220%20-791.3%20888.5%20936.9%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3EN%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-4E%22%20d%3D%22M234%20637Q231%20637%20226%20637Q201%20637%20196%20638T191%20649Q191%20676%20202%20682Q204%20683%20299%20683Q376%20683%20387%20683T401%20677Q612%20181%20616%20168L670%20381Q723%20592%20723%20606Q723%20633%20659%20637Q635%20637%20635%20648Q635%20650%20637%20660Q641%20676%20643%20679T653%20683Q656%20683%20684%20682T767%20680Q817%20680%20843%20681T873%20682Q888%20682%20888%20672Q888%20650%20880%20642Q878%20637%20858%20637Q787%20633%20769%20597L620%207Q618%200%20599%200Q585%200%20582%202Q579%205%20453%20305L326%20604L261%20344Q196%2088%20196%2079Q201%2046%20268%2046H278Q284%2041%20284%2038T282%2019Q278%206%20272%200H259Q228%202%20151%202Q123%202%20100%202T63%202T46%201Q31%201%2031%2010Q31%2014%2034%2026T39%2040Q41%2046%2062%2046Q130%2049%20150%2085Q154%2091%20221%20362L289%20634Q287%20635%20234%20637Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-4E%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)轴）旋转![\beta](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%221.332ex%22%20height%3D%222.509ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.671ex%3B%22%20viewBox%3D%220%20-791.3%20573.5%201080.4%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3E%5Cbeta%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-3B2%22%20d%3D%22M29%20-194Q23%20-188%2023%20-186Q23%20-183%20102%20134T186%20465Q208%20533%20243%20584T309%20658Q365%20705%20429%20705H431Q493%20705%20533%20667T573%20570Q573%20465%20469%20396L482%20383Q533%20332%20533%20252Q533%20139%20448%2065T257%20-10Q227%20-10%20203%20-2T165%2017T143%2040T131%2059T126%2065L62%20-188Q60%20-194%2042%20-194H29ZM353%20431Q392%20431%20427%20419L432%20422Q436%20426%20439%20429T449%20439T461%20453T472%20471T484%20495T493%20524T501%20560Q503%20569%20503%20593Q503%20611%20502%20616Q487%20667%20426%20667Q384%20667%20347%20643T286%20582T247%20514T224%20455Q219%20439%20186%20308T152%20168Q151%20163%20151%20147Q151%2099%20173%2068Q204%2026%20260%2026Q302%2026%20349%2051T425%20137Q441%20171%20449%20214T457%20279Q457%20337%20422%20372Q380%20358%20347%20358H337Q258%20358%20258%20389Q258%20396%20261%20403Q275%20431%20353%20431Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-3B2%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)角
- 最后绕**自己**的![Z](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%221.68ex%22%20height%3D%222.176ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.338ex%3B%22%20viewBox%3D%220%20-791.3%20723.5%20936.9%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3EZ%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-5A%22%20d%3D%22M58%208Q58%2023%2064%2035Q64%2036%20329%20334T596%20635L586%20637Q575%20637%20512%20637H500H476Q442%20637%20420%20635T365%20624T311%20598T266%20548T228%20469Q227%20466%20226%20463T224%20458T223%20453T222%20450L221%20448Q218%20443%20202%20443Q185%20443%20182%20453L214%20561Q228%20606%20241%20651Q249%20679%20253%20681Q256%20683%20487%20683H718Q723%20678%20723%20675Q723%20673%20717%20649Q189%2054%20188%2052L185%2049H274Q369%2050%20377%2051Q452%2060%20500%20100T579%20247Q587%20272%20590%20277T603%20282H607Q628%20282%20628%20271Q547%205%20541%202Q538%200%20300%200H124Q58%200%2058%208Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-5A%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)轴旋转![\gamma](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%221.262ex%22%20height%3D%222.176ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.838ex%3B%22%20viewBox%3D%220%20-576.1%20543.5%20936.9%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3E%5Cgamma%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-3B3%22%20d%3D%22M31%20249Q11%20249%2011%20258Q11%20275%2026%20304T66%20365T129%20418T206%20441Q233%20441%20239%20440Q287%20429%20318%20386T371%20255Q385%20195%20385%20170Q385%20166%20386%20166L398%20193Q418%20244%20443%20300T486%20391T508%20430Q510%20431%20524%20431H537Q543%20425%20543%20422Q543%20418%20522%20378T463%20251T391%2071Q385%2055%20378%206T357%20-100Q341%20-165%20330%20-190T303%20-216Q286%20-216%20286%20-188Q286%20-138%20340%2032L346%2051L347%2069Q348%2079%20348%20100Q348%20257%20291%20317Q251%20355%20196%20355Q148%20355%20108%20329T51%20260Q49%20251%2047%20251Q45%20249%2031%20249Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-3B3%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)角



**很显然，按照不同的旋转步骤，旋转的结果是不一样的。**

就好比问路的时候，回答你，“左转再右转”，和“右转再左转”，肯定到达的地点是不一样的。

我们需要把上面的旋转步骤按照顺序标记为![zXZ](data:image/svg+xml;utf8,%3Csvg%20xmlns%3Axlink%3D%22http%3A%2F%2Fwww.w3.org%2F1999%2Fxlink%22%20class%3D%22mjx-svg-math%22%20width%3D%224.749ex%22%20height%3D%222.176ex%22%20style%3D%22font-size%3A14px%3Bvertical-align%3A%20-0.338ex%3B%22%20viewBox%3D%220%20-791.3%202044.5%20936.9%22%20role%3D%22img%22%20focusable%3D%22false%22%20xmlns%3D%22http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg%22%20aria-labelledby%3D%22MathJax-SVG-1-Title%22%3E%0A%3Ctitle%20id%3D%22MathJax-SVG-1-Title%22%3EzXZ%3C%2Ftitle%3E%0A%3Cdefs%20aria-hidden%3D%22true%22%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-7A%22%20d%3D%22M347%20338Q337%20338%20294%20349T231%20360Q211%20360%20197%20356T174%20346T162%20335T155%20324L153%20320Q150%20317%20138%20317Q117%20317%20117%20325Q117%20330%20120%20339Q133%20378%20163%20406T229%20440Q241%20442%20246%20442Q271%20442%20291%20425T329%20392T367%20375Q389%20375%20411%20408T434%20441Q435%20442%20449%20442H462Q468%20436%20468%20434Q468%20430%20463%20420T449%20399T432%20377T418%20358L411%20349Q368%20298%20275%20214T160%20106L148%2094L163%2093Q185%2093%20227%2082T290%2071Q328%2071%20360%2090T402%20140Q406%20149%20409%20151T424%20153Q443%20153%20443%20143Q443%20138%20442%20134Q425%2072%20376%2031T278%20-11Q252%20-11%20232%206T193%2040T155%2057Q111%2057%2076%20-3Q70%20-11%2059%20-11H54H41Q35%20-5%2035%20-2Q35%2013%2093%2084Q132%20129%20225%20214T340%20322Q352%20338%20347%20338Z%22%3E%3C%2Fpath%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-58%22%20d%3D%22M42%200H40Q26%200%2026%2011Q26%2015%2029%2027Q33%2041%2036%2043T55%2046Q141%2049%20190%2098Q200%20108%20306%20224T411%20342Q302%20620%20297%20625Q288%20636%20234%20637H206Q200%20643%20200%20645T202%20664Q206%20677%20212%20683H226Q260%20681%20347%20681Q380%20681%20408%20681T453%20682T473%20682Q490%20682%20490%20671Q490%20670%20488%20658Q484%20643%20481%20640T465%20637Q434%20634%20411%20620L488%20426L541%20485Q646%20598%20646%20610Q646%20628%20622%20635Q617%20635%20609%20637Q594%20637%20594%20648Q594%20650%20596%20664Q600%20677%20606%20683H618Q619%20683%20643%20683T697%20681T738%20680Q828%20680%20837%20683H845Q852%20676%20852%20672Q850%20647%20840%20637H824Q790%20636%20763%20628T722%20611T698%20593L687%20584Q687%20585%20592%20480L505%20384Q505%20383%20536%20304T601%20142T638%2056Q648%2047%20699%2046Q734%2046%20734%2037Q734%2035%20732%2023Q728%207%20725%204T711%201Q708%201%20678%201T589%202Q528%202%20496%202T461%201Q444%201%20444%2010Q444%2011%20446%2025Q448%2035%20450%2039T455%2044T464%2046T480%2047T506%2054Q523%2062%20523%2064Q522%2064%20476%20181L429%20299Q241%2095%20236%2084Q232%2076%20232%2072Q232%2053%20261%2047Q262%2047%20267%2047T273%2046Q276%2046%20277%2046T280%2045T283%2042T284%2035Q284%2026%20282%2019Q279%206%20276%204T261%201Q258%201%20243%201T201%202T142%202Q64%202%2042%200Z%22%3E%3C%2Fpath%3E%0A%3Cpath%20stroke-width%3D%221%22%20id%3D%22E1-MJMATHI-5A%22%20d%3D%22M58%208Q58%2023%2064%2035Q64%2036%20329%20334T596%20635L586%20637Q575%20637%20512%20637H500H476Q442%20637%20420%20635T365%20624T311%20598T266%20548T228%20469Q227%20466%20226%20463T224%20458T223%20453T222%20450L221%20448Q218%20443%20202%20443Q185%20443%20182%20453L214%20561Q228%20606%20241%20651Q249%20679%20253%20681Q256%20683%20487%20683H718Q723%20678%20723%20675Q723%20673%20717%20649Q189%2054%20188%2052L185%2049H274Q369%2050%20377%2051Q452%2060%20500%20100T579%20247Q587%20272%20590%20277T603%20282H607Q628%20282%20628%20271Q547%205%20541%202Q538%200%20300%200H124Q58%200%2058%208Z%22%3E%3C%2Fpath%3E%0A%3C%2Fdefs%3E%0A%3Cg%20stroke%3D%22currentColor%22%20fill%3D%22currentColor%22%20stroke-width%3D%220%22%20transform%3D%22matrix(1%200%200%20-1%200%200)%22%20aria-hidden%3D%22true%22%3E%0A%3Cg%20class%3D%22mjx-svg-mrow%22%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-7A%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%20transform%3D%22translate(468%2C0)%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-58%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3Cg%20class%3D%22mjx-svg-mi%22%20transform%3D%22translate(1321%2C0)%22%3E%0A%20%3Cuse%20xlink%3Ahref%3D%22%23E1-MJMATHI-5A%22%3E%3C%2Fuse%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fg%3E%0A%3C%2Fsvg%3E)，加上角度就是一个完整的欧拉角



#### 万向节死锁（自由度下降）——为什么要用四元数

------



## 




