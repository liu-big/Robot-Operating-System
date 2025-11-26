# -*- coding: utf-8 -*-
#
# tf2 文档构建配置文件，由 sphinx-quickstart 在 2009 年 6 月 1 日 14:21:53 创建。
#
# 此文件在当前目录设置为其包含目录时执行。
#
# 请注意，此自动生成的文件中并非包含所有可能的配置值。
#
# 所有配置值都有默认值；被注释掉的值用于显示默认值。

import roslib
#roslib.load_manifest('tf2_kdl') # 加载 tf2_kdl 的 ROS 库清单
import sys, os

# 如果扩展（或要使用 autodoc 文档化的模块）位于另一个目录中，
# 请在此处将这些目录添加到 sys.path。如果目录是相对于文档根目录的，
# 请使用 os.path.abspath 使其成为绝对路径，如下所示。
# sys.path.append(os.path.abspath('./src/tf2_kdl'))

# -- 通用配置 ------------------------------------------------------------------

# 在此处添加任何 Sphinx 扩展模块名称，作为字符串。它们可以是 Sphinx 自带的扩展
#（命名为 'sphinx.ext.*'）或您的自定义扩展。
extensions = ['sphinx.ext.autodoc', 'sphinx.ext.doctest', 'sphinx.ext.intersphinx', 'sphinx.ext.pngmath']

# 在此处添加任何包含模板的路径，相对于此目录。
templates_path = ['_templates']

# 源文件名的后缀。
source_suffix = '.rst'

# 源文件的编码。
#source_encoding = 'utf-8'

# 主目录树文档。
master_doc = 'index'

# 有关项目的通用信息。
project = u'tf2_kdl'
copyright = u'2016, Open Source Robotics Foundation'

# 文档项目版本信息，用于替换 |version| 和 |release|，
# 也在构建文档的其他地方使用。
#
# 短版本号 X.Y。
version = '0.5'
# 完整版本号，包括 alpha/beta/rc 标签。
release = '0.5.13'

# Sphinx 自动生成内容的语言。请参阅文档以获取支持语言列表。
#language = None

# 替换 |today| 有两种选择：
# 1. 将 today 设置为非假值，然后使用它：
#today = ''
# 2. 否则，today_fmt 用作 strftime 调用的格式。
#today_fmt = '%B %d, %Y'

# 不应包含在构建中的文档列表。
#unused_docs = []

# 相对于源目录，不应搜索源文件的目录列表。
exclude_trees = ['_build']

exclude_patterns = ['_CHANGELOG.rst']

# 用于所有文档的 reST 默认角色（用于此标记：`text`）。
#default_role = None

# 如果为 True，'()' 将附加到 :func: 等交叉引用文本。
#add_function_parentheses = True

# 如果为 True，当前模块名称将添加到所有描述单元标题（例如 .. function::）的前面。
#add_module_names = True

# 如果为 True，将在输出中显示 sectionauthor 和 moduleauthor 指令。它们默认被忽略。
#show_authors = False

# 要使用的 Pygments（语法高亮）样式名称。
pygments_style = 'sphinx'

# 模块索引排序时忽略的前缀列表。
#modindex_common_prefix = []


# -- HTML 输出选项 -----------------------------------------------------------

# 用于 HTML 和 HTML 帮助页面的主题。Sphinx 附带的主要主题目前是 'default' 和 'sphinxdoc'。
html_theme = 'default'

# 主题选项是主题特定的，并自定义主题的外观和感觉。
# 有关每个主题可用选项的列表，请参阅文档。
#html_theme_options = {}

# 在此处添加任何包含自定义主题的路径，相对于此目录。
#html_theme_path = []

# 此组 Sphinx 文档的名称。如果为 None，则默认为 "<project> v<release> documentation"。
#html_title = None

# 导航栏的较短标题。默认为与 html_title 相同。
#html_short_title = None

# 要放置在侧边栏顶部的图像文件（相对于此目录）的名称。
#html_logo = None

# 要用作文档 favicon 的图像文件（在静态路径内）的名称。
# 此文件应为 16x16 或 32x32 像素的 Windows 图标文件 (.ico)。
#html_favicon = None

# 在此处添加任何包含自定义静态文件（如样式表）的路径，相对于此目录。
# 它们在内置静态文件之后复制，因此名为 "default.css" 的文件将覆盖内置的 "default.css"。
#html_static_path = ['_static']

# 如果不为空，则在每个页面底部插入 'Last updated on:' 时间戳，使用给定的 strftime 格式。
#html_last_updated_fmt = '%b %d, %Y'

# 如果为 True，将使用 SmartyPants 将引号和破折号转换为排版正确的实体。
#html_use_smarty_pants = True

# 自定义侧边栏模板，将文档名称映射到模板名称。
#html_sidebars = {}

# 应渲染到页面的其他模板，将页面名称映射到模板名称。
#html_additional_pages = {}

# 如果为 False，则不生成模块索引。
#html_use_modindex = True

# 如果为 False，则不生成索引。
#html_use_index = True

# 如果为 True，索引将拆分为每个字母的单独页面。
#html_split_index = False

# 如果为 True，则将 reST 源的链接添加到页面。
#html_show_sourcelink = True

# 如果为 True，将输出 OpenSearch 描述文件，并且所有页面都将包含引用它的 <link> 标签。
# 此选项的值必须是提供完成的 HTML 的基本 URL。
#html_use_opensearch = ''

# 如果不为空，这是 HTML 文件的文件名前缀（例如 ".xhtml"）。
#html_file_suffix = ''

# HTML 帮助构建器的输出文件基本名称。
htmlhelp_basename = 'tfdoc'


# -- LaTeX 输出选项 ----------------------------------------------------------

# 纸张大小（'letter' 或 'a4'）。
#latex_paper_size = 'letter'

# 字体大小（'10pt', '11pt' 或 '12pt'）。
#latex_font_size = '10pt'

# 将文档树分组到 LaTeX 文件中。元组列表
# (源启动文件, 目标名称, 标题, 作者, 文档类 [howto/manual])。
latex_documents = [
  ('index', 'tf.tex', u'stereo\\_utils Documentation',
   u'Tully Foote and Eitan Marder-Eppstein', 'manual'),
]

# 要放置在标题页顶部的图像文件（相对于此目录）的名称。
#latex_logo = None

# 对于 "manual" 文档，如果为 True，则顶级标题是部分，而不是章节。
#latex_use_parts = False

# LaTeX 序言的其他内容。
#latex_preamble = ''

# 要作为附录附加到所有手册的文档。
#latex_appendices = []

# 如果为 False，则不生成模块索引。
#latex_use_modindex = True


# intersphinx 的示例配置：引用 Python 标准库。
intersphinx_mapping = {
    'http://docs.python.org/': None,
    'http://docs.opencv.org/3.0-last-rst/': None,
    'http://docs.scipy.org/doc/numpy' : None
    }
